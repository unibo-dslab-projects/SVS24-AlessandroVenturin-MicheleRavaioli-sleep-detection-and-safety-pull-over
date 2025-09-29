from enum import StrEnum, auto
from typing import override

from carla import Location, Vector3D, Vehicle, VehicleControl
import pygame

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
from agents.navigation.basic_agent import BasicAgent



class VehicleData:
    enable_logging: bool
    destination: Location

    speed: Vector3D = Vector3D()
    should_enter_manual_driving: bool = False

    vehicle_actor: Vehicle
    vehicle_control: VehicleControl = VehicleControl()
    """
    The Vehicle control to be applied at the end of each step
    """
    pygame_io: PygameIO
    manual_control: PygameVehicleControl
    dashboard_buttons: PygameDashboardButtons = PygameDashboardButtons()
    pygame_events: list[pygame.event.Event] = []
    lane_keeping_agent: BasicAgent

    def __init__(
            self, pygame_io: PygameIO, vehicle_actor: Vehicle, destination:Location, enable_logging: bool = False
    ):
        self.enable_logging = enable_logging
        self.destination = destination
        self.vehicle_actor = vehicle_actor
        self.lane_keeping_agent = BasicAgent(self.vehicle_actor)
        self.pygame_io = pygame_io
        self.manual_control = PygameVehicleControl(vehicle_actor)

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
        vehicle_actor: Vehicle,
        destination: Location,
        enable_logging: bool = False,
    ):
        super().__init__(
            [WrapperS()],
            VehicleData(
                pygame_io=pygame_io,
                vehicle_actor=vehicle_actor,
                destination=destination,
                enable_logging=enable_logging,
            ),
        )
        if self._data.enable_logging:
            self._log_current_state()

    @override
    def step(self, dt: float) -> bool:
        result = super().step(dt)
        if self._data.enable_logging:
            self._log_current_state()
            self._log_data()
        return result

    def _log_current_state(self):
        print(f"current state: {self._state.__class__.__name__}")

    def _log_data(self):
        print(f"throttle: {self._data.vehicle_control.throttle}")
        print(f"brake: {self._data.vehicle_control.brake}")
        print(f"steer: {self._data.vehicle_control.steer}")


class WrapperS(State[VehicleData, VehicleTimers]):
    @override
    def children(self) -> list[VehicleState]:
        return [ManualDrivingS(), CruiseControlS(), ExitS()]

    @override
    def on_early_do(self, data: VehicleData, ctx: VehicleContext):
        data.speed = data.vehicle_actor.get_velocity()
        data.vehicle_control = VehicleControl()
        data.pygame_events = data.pygame_io.update()
        data.manual_control.update(data.pygame_events)
        data.dashboard_buttons.update(data.pygame_events)

    @override
    def on_late_do(self, data: VehicleData, ctx: VehicleContext):
        data.vehicle_actor.apply_control(data.vehicle_control)

    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=ExitS(),
                condition=lambda data, ctx: self._exit_transition_condition(data),
            )
        ]

    def _is_quit_event(self, e: pygame.event.Event) -> bool:
        return e.type == pygame.QUIT

    def _exit_transition_condition(self, data: VehicleData) -> bool:
        return next(filter(self._is_quit_event, data.pygame_events), None) is not None


# ========== EXIT_STATE ==========


class ExitS(VehicleState):
    @override
    def is_exit_state(self) -> bool:
        return True


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


# ========== MANUAL_DRIVING ==========


class ManualDrivingS(VehicleState):
    @override
    def on_do(self, data: VehicleData, ctx: VehicleContext):
        data.vehicle_control = data.manual_control.vehicle_control

    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=LaneKeepingS(),
                condition=lambda data,
                ctx: data.dashboard_buttons.lane_keeping_button_pressed,
            )
        ]


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
            VehicleTransition(
                to=ManualDrivingS(),
                condition=lambda data,
                ctx: data.dashboard_buttons.lane_keeping_button_pressed,
            ),
            # This transition is here for demonstration purposes
            VehicleTransition(
                to=PullOverPreparationS(),
                condition=lambda data,
                ctx: data.dashboard_buttons.force_pullover_button_pressed,
            ),
        ]

    @override
    def on_entry(self, data: VehicleData, ctx: VehicleContext):
        data.lane_keeping_agent.set_destination(data.destination)
        return

    @override
    def on_do(self, data: VehicleData, ctx: VehicleContext):
        data.vehicle_control = data.lane_keeping_agent.run_step()


def _inattention_detected(data: VehicleData) -> bool:
    # TODO
    return False


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
    def on_do(self, data: VehicleData, ctx: VehicleContext):
        # TODO: choose appropriate speed and braking power
        if data.speed.length() > 50 / 3.6:
            data.vehicle_control.brake = 0.2


# ========== PULLING_OVER ==========


class PullingOverS(VehicleState):
    @override
    def children(self) -> list[VehicleState]:
        return [EmergencyLaneNotReachedS(), EmergencyLaneReachedS()]

    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=ManualDrivingS(),
                condition=lambda data, ctx: data.should_enter_manual_driving,
            )
        ]


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
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=ManualDrivingS(),
                condition=lambda data, ctx: data.should_enter_manual_driving,
            )
        ]

    @override
    def on_entry(self, data: VehicleData, ctx: VehicleContext):
        # TODO: activate emergency signals
        ...
