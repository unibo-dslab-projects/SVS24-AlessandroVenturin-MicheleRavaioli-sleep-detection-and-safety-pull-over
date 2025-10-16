from enum import StrEnum, auto
import math
from typing import cast, override

import pygame
from carla import (
    AckermannControllerSettings,
    LaneType,
    Location,
    Map,
    Transform,
    Vector3D,
    Vehicle,
    VehicleAckermannControl,
    VehicleControl,
    Waypoint,
    World,
)

from agents.navigation.basic_agent import BasicAgent
from agents.navigation.global_route_planner import GlobalRoutePlanner
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


class VehicleParams:
    destination: Location
    sensors_max_range: float
    """
    Maximum forward range for sensors regarding safety pull over.
    """
    cruise_target_speed_kmh: float
    """
    Speed to target for the cruise control system
    Must be >= 0.
    """
    pull_over_acceleration: float
    """
    Target braking acceleration for the vehicle while pulling over.
    Must be < 0.
    """
    max_pull_over_preparation_speed_kmh: float
    """
    Maximum speed at which the vehicle can start pulling over in order to
    stop withing the window allowed by the sensors range
    """
    min_pull_over_speed_kmh: float
    """
    Minimum speed to keep while performin the pull over.
    """

    def __init__(
        self,
        destination: Location,
        sensors_max_range: float,
        cruise_target_speed_kmh: float,
        pull_over_acceleration: float,
        min_pull_over_speed_kmh: float,
    ):
        self.destination = destination
        self.sensors_max_range = sensors_max_range
        if cruise_target_speed_kmh < 0:
            raise Exception("cruise target speed must be positive or 0")
        self.cruise_target_speed_kmh = cruise_target_speed_kmh
        if pull_over_acceleration >= 0:
            raise Exception("pull over acceleration must be negative")
        self.pull_over_acceleration = pull_over_acceleration
        self.max_pull_over_preparation_speed_kmh = (
            math.sqrt(2 * -self.pull_over_acceleration * self.sensors_max_range) * 3.6
        )
        if min_pull_over_speed_kmh > self.max_pull_over_preparation_speed_kmh:
            raise Exception(
                "Incompatible parameters, minimum pull over speed is too high"
            )
        self.min_pull_over_speed_kmh = min_pull_over_speed_kmh


class VehicleData:
    logging_config: VehicleLoggingConfig
    world: World
    map: Map
    params: VehicleParams

    acceleration: float = 0
    last_step_speed: Vector3D = Vector3D()
    speed: Vector3D = Vector3D()

    @property
    def speed_kmh(self) -> float:
        return self.speed.length() * 3.6

    vehicle: Vehicle
    vehicle_control: VehicleControl = VehicleControl()
    vehicle_ackermann_control: VehicleAckermannControl | None = None
    last_step_vehicle_control: VehicleControl = VehicleControl()
    """
    The Vehicle control to be applied at the end of each step
    """
    pygame_io: PygameIO
    manual_control: PygameVehicleControl
    dashboard_buttons: PygameDashboardButtons = PygameDashboardButtons()
    pygame_events: list[pygame.event.Event] = []
    cruise_control_agent: BasicAgent
    global_route_planner: GlobalRoutePlanner

    inattention_detector: InattentionDetector
    """
    Detector used to spot inattentive behaviours in the driver.
    """

    def __init__(
        self,
        pygame_io: PygameIO,
        vehicle: Vehicle,
        world: World,
        map: Map,
        params: VehicleParams,
        driver_camera_stream: CameraStream,
        logging_config: VehicleLoggingConfig | None,
    ):
        if logging_config is None:
            self.logging_config = VehicleLoggingConfig()
        else:
            self.logging_config = logging_config
        self.params = params
        self.world = world
        self.map = map
        self.vehicle = vehicle
        self.cruise_control_agent = BasicAgent(self.vehicle, map_inst=map)
        self.global_route_planner = self.cruise_control_agent.get_global_planner()
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
        world: World,
        map: Map,
        params: VehicleParams,
        driver_camera_stream: CameraStream,
        logging_config: VehicleLoggingConfig | None = None,
    ):
        super().__init__(
            [WrapperS()],
            VehicleData(
                pygame_io=pygame_io,
                vehicle=vehicle,
                world=world,
                map=map,
                params=params,
                driver_camera_stream=driver_camera_stream,
                logging_config=logging_config,
            ),
            logging_config=logging_config,
        )

    @override
    def step(self, dt: float) -> bool:
        result = super().step(dt)
        if self._vehicle_logging_config().log_main_vehicle_controls:
            control = self._data.vehicle.get_control()
            self._vehicle_log("accel:", self._data.acceleration)
            self._vehicle_log("speed:", self._data.speed.length() * 3.6, "km/h")
            self._vehicle_log("throt:", control.throttle)
            self._vehicle_log("brake:", control.brake)
            self._vehicle_log("steer:", control.steer)
            self._vehicle_log("gear :", control.gear)
            print()
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
        data.last_step_speed = data.speed
        data.speed = data.vehicle.get_velocity()
        data.acceleration = (
            data.speed.length() - data.last_step_speed.length()
        ) / ctx.dt
        data.last_step_vehicle_control = data.vehicle_control
        data.vehicle_control = VehicleControl()
        data.vehicle_ackermann_control = None
        data.pygame_events = data.pygame_io.update()
        data.manual_control.update(data.pygame_events)
        data.dashboard_buttons.update(data.pygame_events)

    @override
    def on_late_do(self, data: VehicleData, ctx: VehicleContext):
        data.vehicle.apply_control(data.vehicle_control)
        if data.vehicle_ackermann_control is not None:
            data.vehicle.apply_ackermann_control(data.vehicle_ackermann_control)  # pyright: ignore[reportUnknownMemberType]

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

    @override
    def on_entry(self, data: VehicleData, ctx: VehicleContext):
        # Using cached map and global_route_planner to avoid expensive blocking call
        data.cruise_control_agent = BasicAgent(
            data.vehicle, map_inst=data.map, grp_inst=data.global_route_planner
        )
        data.cruise_control_agent.ignore_traffic_lights()
        data.cruise_control_agent.follow_speed_limits(False)
        data.cruise_control_agent.set_target_speed(data.params.cruise_target_speed_kmh)  # pyright: ignore[reportUnknownMemberType]
        data.cruise_control_agent.set_destination(data.params.destination)

    @override
    def on_do(self, data: VehicleData, ctx: VehicleContext):
        data.vehicle_control = data.cruise_control_agent.run_step()


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
        ctx.timer(VehicleTimers.INATTENTION).reset(20)


def _pull_over_is_safe(data: VehicleData) -> bool:
    # TODO add check of obstacles
    return not _junction_detected(data)


def _waypoints_roughly_same_direction(w1: Waypoint, w2: Waypoint) -> bool:
    return w1.transform.get_forward_vector().dot(w2.transform.get_forward_vector()) > 0


def _waypoints_same_road(w1: Waypoint, w2: Waypoint) -> bool:
    return w1.road_id == w2.road_id


def _junction_detected(data: VehicleData) -> bool:
    # We use the target waypoint as getting a waypoint under the vehicle position
    # may return a waypoint that is part of another overlapping lane
    target_waypoint = cast(
        Waypoint, data.cruise_control_agent.get_local_planner().target_waypoint
    )

    # Here we find a waypoint that is in the exact position of the car but on the correct lane
    distance_from_car = target_waypoint.transform.location.distance(
        data.vehicle.get_location()
    )
    curr_waypoint = next(
        filter(
            lambda w: w.lane_id == target_waypoint.lane_id,
            target_waypoint.previous(distance_from_car),
        )
    )

    # Here we compute N waypoints ahead of the vehicle to check if they are part of a junction
    waypoints = [curr_waypoint]

    # Here get waypoints ahead of the vehicle at least every 10 meters up to they
    # specified distance. Highway junctions in carla are never shorter than 10 meters
    # We choose a large interval in order to not overload the simulation, but in real life
    # we would prioritize safety and reduce this interval
    meters_ahead = 0
    if data.params.sensors_max_range / 3 > 10:
        increment = 10
    else:
        increment = data.params.sensors_max_range / 3
    while meters_ahead < data.params.sensors_max_range:
        # Ensuring not to exceed the maximum range
        meters_ahead = min(meters_ahead + increment, data.params.sensors_max_range)
        w = next(
            filter(
                lambda w: w.lane_id == curr_waypoint.lane_id,
                curr_waypoint.next(meters_ahead),
            )
        )
        waypoints.append(w)

    for w in waypoints:
        # Junctions are road segments and as such they include both sides, so checking if
        # the waypoint is part of a junction is not enough because we don't care if the
        # actual exit or entry is on the other side of the road
        if w.is_junction:
            junction_waypoints = w.get_junction().get_waypoints(LaneType.Any)

            # Here we take just those waypoints that point to the same direction as the vehicle
            junction_waypoints = filter(
                lambda t: _waypoints_roughly_same_direction(t[0], w),
                junction_waypoints,
            )

            # Here we search for a waypoint that is part of a different road than the vehicle one
            waypoint_in_different_road = next(
                filter(
                    lambda t: not _waypoints_same_road(t[0], w)
                    or not _waypoints_same_road(t[1], w),
                    junction_waypoints,
                ),
                None,
            )

            # If there is at least one waypoint that is part of a different road then
            # we assume there is an actual junction on the vehicle side of the road
            return waypoint_in_different_road is not None

    return False


class PullOverPreparationS(VehicleState):
    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=PullingOverS(),
                condition=lambda data, ctx: (
                    _pull_over_is_safe(data)
                    and data.speed_kmh
                    <= data.params.max_pull_over_preparation_speed_kmh
                ),
            )
        ]

    @override
    def on_entry(self, data: VehicleData, ctx: VehicleContext):
        data.cruise_control_agent.set_target_speed(  # pyright: ignore[reportUnknownMemberType]
            data.params.max_pull_over_preparation_speed_kmh
        )

    @override
    def actions(self) -> list[VehicleStateAction]:
        return [
            VehicleStateAction(
                condition=lambda data, ctx: _junction_detected(data),
                action=lambda data, ctx: data.world.debug.draw_string(
                    data.vehicle.get_location() + Location(z=2), "Junction detected"
                ),
            )
        ]


# ========== PULLING_OVER ==========


class PullingOverS(VehicleState):
    @override
    def children(self) -> list[VehicleState]:
        return [EmergencyLaneNotReachedS(), EmergencyLaneReachedS()]


def _signed_lateral_distance(of: Location, to: Transform) -> float:
    target_rotation = to.rotation
    yaw_rad = math.radians(target_rotation.yaw)
    right_vector = Vector3D(-math.sin(yaw_rad), math.cos(yaw_rad), 0)
    offset_vector = of - to.location
    return offset_vector.dot(right_vector)


def _vehicle_front(data: VehicleData) -> Location:
    vehicle_t = data.vehicle.get_transform()
    vehicle_half_lenght = data.vehicle.bounding_box.extent.x
    return Location(
        vehicle_t.location + vehicle_t.get_forward_vector() * vehicle_half_lenght
    )


def _emergency_lane_reached(data: VehicleData) -> bool:
    vehicle_front = _vehicle_front(data)
    waypoint = data.map.get_waypoint(vehicle_front, lane_type=LaneType.Any)
    signed_lateral_distance = _signed_lateral_distance(
        vehicle_front, waypoint.transform
    )
    return waypoint.lane_type == LaneType.Shoulder and signed_lateral_distance >= 0


def _clamp(a: float, min_: float, max_: float) -> float:
    return min(max(a, min_), max_)


def _steer_to_radians(data: VehicleData, steer: float) -> float:
    max_steer_angle_deg = data.vehicle.get_physics_control().wheels[0].max_steer_angle
    steer = _clamp(steer, -1, 1)
    return math.radians(max_steer_angle_deg) * steer


def _keep_target_speed(data: VehicleData, speed_kmh: float):
    speed_ms = speed_kmh / 3.6
    if (
        data.speed.length() > speed_ms + 1
    ):  # a bit of tolerance to avoid oscillating behavior
        data.vehicle.apply_ackermann_controller_settings(  # pyright: ignore[reportUnknownMemberType]
            AckermannControllerSettings(  # pyright: ignore[reportArgumentType]
                speed_kp=0.15,
                speed_ki=0.0,
                speed_kd=0.25,
                accel_kp=1.0,
                accel_ki=0.0,
                accel_kd=0.2,
            )
        )
        acc = data.params.pull_over_acceleration
    else:
        # Reset settings to default
        data.vehicle.apply_ackermann_controller_settings(  # pyright: ignore[reportUnknownMemberType]
            AckermannControllerSettings(  # pyright: ignore[reportArgumentType]
                speed_kp=0.15,
                speed_ki=0.0,
                speed_kd=0.25,
                accel_kp=0.01,
                accel_ki=0.0,
                accel_kd=0.01,
            )
        )
        acc = 0
    data.vehicle_ackermann_control = VehicleAckermannControl(
        acceleration=acc, speed=speed_ms
    )


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
        _keep_target_speed(data, data.params.min_pull_over_speed_kmh)

        # Should be roughly between 0 and 1
        speed_coeff = data.speed_kmh / 50

        # TODO: activate turn signals
        data.vehicle_ackermann_control.steer = _steer_to_radians(
            data, 0.1 * (1 - speed_coeff)
        )


class EmergencyLaneReachedS(VehicleState):
    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=StoppedS(),
                condition=lambda data, ctx: data.speed_kmh <= 0,
            )
        ]

    @override
    def on_do(self, data: VehicleData, ctx: VehicleContext):
        waypoint = data.map.get_waypoint(data.vehicle.get_location())
        if (
            waypoint.transform.get_forward_vector().dot(
                data.vehicle.get_transform().get_forward_vector()
            )
            < 0.999
        ):
            _keep_target_speed(data, data.params.min_pull_over_speed_kmh)
        else:
            _keep_target_speed(data, 0)

        # Should be roughly between 0 and 1
        speed_coeff = data.speed_kmh / 50

        vehicle_front = _vehicle_front(data)
        lane_w = data.map.get_waypoint(vehicle_front, lane_type=LaneType.Any)
        data.vehicle_ackermann_control.steer = _steer_to_radians(
            data,
            -_signed_lateral_distance(vehicle_front, lane_w.transform)
            * (1 - speed_coeff),
        )


# ========== STOPPED ==========


class StoppedS(VehicleState):
    @override
    def on_entry(self, data: VehicleData, ctx: VehicleContext):
        # TODO: activate emergency signals
        ...
