from enum import StrEnum, auto
from typing import cast, override

import pygame
from carla import Location, Map, Vector3D, Vehicle, VehicleControl

from agents.navigation.basic_agent import BasicAgent
from inattention.detector import InattentionDetector
from inattention.utils import CameraStream
from pygame_dashboard_buttons import PygameDashboardButtons
from pygame_io import PygameIO
from pygame_vehicle_control import PygameVehicleControl
from state_machine.sync_state_machine import (
    Context,
    State,
    StateAction,
    SyncStateMachine,
    Transition,
)
from vehicle_logging_config import VehicleLoggingConfig


class VehicleData:
    logging_config: VehicleLoggingConfig
    destination: Location
    map: Map
    meters_for_safe_pullover: float

    speed: Vector3D = Vector3D()

    vehicle: Vehicle
    vehicle_control: VehicleControl = VehicleControl()
    """
    The Vehicle control to be applied at the end of each step
    """
    pygame_io: PygameIO
    manual_control: PygameVehicleControl
    dashboard_buttons: PygameDashboardButtons = PygameDashboardButtons()
    pygame_events: list[pygame.event.Event] = []
    cruise_control_agent: BasicAgent

    inattention_detector: InattentionDetector
    """
    Detector used to spot inattentive behaviours in the driver.
    """

    def __init__(
        self,
        pygame_io: PygameIO,
        vehicle: Vehicle,
        destination: Location,
        map: Map,
        meters_for_safe_pullover: float,
        driver_camera_stream: CameraStream,
        logging_config: VehicleLoggingConfig | None,
    ):
        if logging_config is None:
            self.logging_config = VehicleLoggingConfig()
        else:
            self.logging_config = logging_config
        self.destination = destination
        self.map = map
        self.meters_for_safe_pullover = meters_for_safe_pullover
        self.vehicle = vehicle
        self.cruise_control_agent = BasicAgent(self.vehicle)
        self.cruise_control_agent.ignore_traffic_lights()
        self.cruise_control_agent.set_target_speed(50)  # pyright: ignore[reportUnknownMemberType]
        self.pygame_io = pygame_io
        self.manual_control = PygameVehicleControl(vehicle)
        self.inattention_detector = InattentionDetector(
            driver_camera_stream, eye_threshold=0.15
        )


class VehicleTimers(StrEnum):
    INATTENTION = auto()


type VehicleContext = Context[VehicleTimers]


class VehicleState(State[VehicleData, VehicleTimers]): ...


class VehicleStateAction(StateAction[VehicleData, VehicleTimers]): ...


class VehicleTransition(Transition[VehicleData, VehicleTimers]): ...


class VehicleStateMachine(SyncStateMachine[VehicleData, VehicleTimers]):
    def __init__(
        self,
        pygame_io: PygameIO,
        vehicle: Vehicle,
        destination: Location,
        map: Map,
        meters_for_safe_pullover: float,
        driver_camera_stream: CameraStream,
        logging_config: VehicleLoggingConfig | None = None,
    ):
        super().__init__(
            [WrapperS()],
            VehicleData(
                pygame_io=pygame_io,
                vehicle=vehicle,
                destination=destination,
                map=map,
                meters_for_safe_pullover=meters_for_safe_pullover,
                driver_camera_stream=driver_camera_stream,
                logging_config=logging_config,
            ),
            logging_config=logging_config,
        )

    @override
    def step(self, dt: float) -> bool:
        result = super().step(dt)
        if self._vehicle_logging_config().log_main_vehicle_controls:
            self._vehicle_log("throttle:", self._data.vehicle_control.throttle)
            self._vehicle_log("brake:", self._data.vehicle_control.brake)
            self._vehicle_log("steer:", self._data.vehicle_control.steer)
        return result

    def _vehicle_log(self, *values: object):
        print("VEHICLE_STATE_MACHINE: ", *values)

    def _vehicle_logging_config(self):
        return cast(VehicleLoggingConfig, self._logging_config)


class WrapperS(State[VehicleData, VehicleTimers]):
    @override
    def children(self) -> list[VehicleState]:
        return [ManualDrivingS(), CruiseControlS(), ExitS()]

    @override
    def on_early_do(self, data: VehicleData, ctx: VehicleContext):
        data.speed = data.vehicle.get_velocity()
        data.vehicle_control = VehicleControl()
        data.pygame_events = data.pygame_io.update()
        data.manual_control.update(data.pygame_events)
        data.dashboard_buttons.update(data.pygame_events)

    @override
    def on_late_do(self, data: VehicleData, ctx: VehicleContext):
        data.vehicle.apply_control(data.vehicle_control)

    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=ExitS(),
                condition=lambda data, ctx: self._exit_transition_condition(data),
            )
        ]

    @override
    def on_exit(self, data: VehicleData, ctx: Context[VehicleTimers]):
        data.inattention_detector.close()

    def _is_quit_event(self, e: pygame.event.Event) -> bool:
        return e.type == pygame.QUIT

    def _exit_transition_condition(self, data: VehicleData) -> bool:
        return next(filter(self._is_quit_event, data.pygame_events), None) is not None


# ========== EXIT_STATE ==========


class ExitS(VehicleState):
    @override
    def is_exit_state(self) -> bool:
        return True


# ========== MANUAL_DRIVING ==========


class ManualDrivingS(VehicleState):
    @override
    def on_do(self, data: VehicleData, ctx: VehicleContext):
        data.vehicle_control = data.manual_control.vehicle_control

    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=CruiseControlS(),
                condition=lambda data,
                ctx: data.dashboard_buttons.cruise_control_button_pressed,
            )
        ]


# ========== CRUISE_CONTROL ==========


class CruiseControlS(VehicleState):
    @override
    def children(self) -> list[VehicleState]:
        return [LaneKeepingS(), PullingOverS(), StoppedS()]

    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=ManualDrivingS(),
                condition=lambda data,
                ctx: data.dashboard_buttons.manual_control_button_pressed,
            )
        ]

    @override
    def on_entry(self, data: VehicleData, ctx: VehicleContext):
        data.cruise_control_agent.set_destination(data.destination)
        return

    @override
    def on_do(self, data: VehicleData, ctx: VehicleContext):
        data.vehicle_control = data.cruise_control_agent.run_step()


# ========== LANE_KEEPING ==========


class LaneKeepingS(VehicleState):
    @override
    def children(self) -> list[VehicleState]:
        return [
            NoInattentionDetectedS(),
            InattentionDetectedS(),
            PullOverPreparationS(),
        ]

    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            # This transition is here for demonstration purposes
            VehicleTransition(
                to=PullOverPreparationS(),
                condition=lambda data,
                ctx: data.dashboard_buttons.force_pullover_button_pressed,
            ),
        ]


def _inattention_detected(data: VehicleData) -> bool:
    return data.inattention_detector.detect()


class NoInattentionDetectedS(VehicleState):
    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=InattentionDetectedS(),
                condition=lambda data, ctx: _inattention_detected(data),
            )
        ]


class InattentionDetectedS(VehicleState):
    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=NoInattentionDetectedS(),
                condition=lambda data, ctx: not _inattention_detected(data),
            ),
            VehicleTransition(
                to=PullOverPreparationS(),
                condition=lambda data, ctx: ctx.timer(
                    VehicleTimers.INATTENTION
                ).is_elapsed(),
            ),
        ]

    @override
    def on_entry(self, data: VehicleData, ctx: VehicleContext):
        # TODO: choose proper amount of seconds
        ctx.timer(VehicleTimers.INATTENTION).reset(5)


def _pull_over_is_safe(data: VehicleData) -> bool:
    # TODO
    return False

def _exit_detected(data: VehicleData)-> bool:
    curr_waypoint = data.map.get_waypoint(data.vehicle.get_location())
    for m in range(1, int(data.meters_for_safe_pullover)):
        available_paths = curr_waypoint.next(m)
        if available_paths.__len__() > 1:
            return True
    return False

class PullOverPreparationS(VehicleState):
    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=PullingOverS(),
                condition=lambda data, ctx: _pull_over_is_safe(data),
            )
        ]

    @override
    def on_entry(self, data: VehicleData, ctx: VehicleContext):
        data.cruise_control_agent.set_target_speed(50)  # pyright: ignore[reportUnknownMemberType]

    @override
    def actions(self) -> list[VehicleStateAction]:
        return [
            VehicleStateAction(
                condition=lambda data, ctx: _exit_detected(data),
                action=lambda data, ctx: print(
                    f"Exit detected in {data.meters_for_safe_pullover} meters"
                ),
            )
        ]

    @override
    def on_do(self, data: VehicleData, ctx: VehicleContext):
        # TODO: choose appropriate speed and braking power
        if data.speed.length() > 50 / 3.6:
            data.vehicle_control.brake = 0.2


# ========== PULLING_OVER ==========


class PullingOverS(VehicleState):
    @override
    def children(self) -> list[VehicleState]:
        return [EmergencyLaneNotReachedS(), EmergencyLaneReachedS()]


def _emergency_lane_reached(data: VehicleData) -> bool:
    # TODO
    return True


class EmergencyLaneNotReachedS(VehicleState):
    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=EmergencyLaneReachedS(),
                condition=lambda data, ctx: _emergency_lane_reached(data),
            )
        ]

    @override
    def on_do(self, data: VehicleData, ctx: VehicleContext):
        # TODO: choose appropriate speed and braking power
        if data.speed.length() > 10 / 3.6:
            data.vehicle_control.brake = 0.2
        # TODO: activate turn signals
        # TODO: amount of steering should be adjusted based on:
        #       - vehicle speed
        #       - how fast the vehicle is approaching the guardrail
        data.vehicle_control.steer = 0.1


class EmergencyLaneReachedS(VehicleState):
    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=StoppedS(),
                condition=lambda data, ctx: data.speed.length() <= 0,
            )
        ]

    @override
    def on_do(self, data: VehicleData, ctx: VehicleContext):
        # TODO: choose appropriate braking power
        data.vehicle_control.brake = 0.2

        # TODO: maybe it would be good to continue adjusting the
        #       steering in order not to exit the emergency lane
        #       or hit the guardrail (especially when pulling over
        #       when the road is turning


# ========== STOPPED ==========


class StoppedS(VehicleState):
    @override
    def on_entry(self, data: VehicleData, ctx: VehicleContext):
        # TODO: activate emergency signals
        ...
