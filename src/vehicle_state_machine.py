import math
from enum import StrEnum, auto
from typing import cast, override

import pygame
from carla import (
    AckermannControllerSettings,
    LaneType,
    Location,
    Map,
    Sensor,
    TrafficManager,
    Transform,
    Vector3D,
    Vehicle,
    VehicleAckermannControl,
    VehicleControl,
    VehicleWheelLocation,
    VehicleLightState,
    Waypoint,
    World,
)

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
from pullover.checker import SafePulloverChecker
from vehicle_logging_config import VehicleLoggingConfig


class VehicleParams:
    sensors_max_range: float
    """
    Maximum forward range for sensors regarding safety pull over.
    """
    cruise_target_speed_kmh: float
    """
    Speed to target for the cruise control system
    Must be >= 0.
    """
    max_pull_over_acceleration: float
    """
    Target braking acceleration for the vehicle while pulling over.
    Must be < 0.
    """
    min_pull_over_speed_kmh: float = 5
    """
    Miminum speed at which the vehicle will move when pulling over
    """
    min_pull_over_space: float = 10
    """
    Even if the vehicle is going really slow this is the minimum amount of space
    needed for a 100% safe pull over.
    """
    radar_scan_width: float

    pull_over_potential_field_coeff: float = 1.35
    road_margin_repulsive_potential_field_coeff: float = 2

    inattention_check_interval: float = 2
    """
    Interval in seconds at which check the driver attention.
    It is used to perform the check over accumulated data instead
    of performing it each frame which is less effective.
    """

    attention_time_percentage: float = 0.7
    """
    A percentage representing the amount of time that the driver needs
    to be paying attention in order to be considered to be paying attention.
    Valid values are >= 0 and <= 1
    """

    wake_up_sound_delay: float = 8

    @property
    def max_pull_over_preparation_speed_kmh(self) -> float:
        """
        Maximum speed at which the vehicle can start pulling over in order to
        stop withing the window allowed by the sensors range
        """
        return (
            math.sqrt(2 * -self.max_pull_over_acceleration * self.sensors_max_range)
            * 3.6
        )

    def __init__(
        self,
        sensors_max_range: float,
        cruise_target_speed_kmh: float,
        max_pull_over_acceleration: float,
        radar_scan_width: float,
    ):
        self.sensors_max_range = sensors_max_range
        if cruise_target_speed_kmh < 0:
            raise Exception("cruise target speed must be positive or 0")
        self.cruise_target_speed_kmh = cruise_target_speed_kmh
        if max_pull_over_acceleration >= 0:
            raise Exception("pull over acceleration must be negative")
        self.max_pull_over_acceleration = max_pull_over_acceleration
        self.radar_scan_width = radar_scan_width


class VehicleData:
    logging_config: VehicleLoggingConfig
    world: World
    map: Map
    topology: list[tuple[Waypoint, Waypoint]]
    traffic_manager: TrafficManager
    params: VehicleParams

    speed: Vector3D = Vector3D()
    pull_over_acceleration: float = 0
    """
    Deceleration that is computed once starting to pull over.
    It is the deceleration that should be used in order to stop in the right amount of space.
    It must not exceed the maximum pull over acceleration given in the params.
    """
    first_junction_distance: float | None = None
    """
    Distance from the first detected junction.
    If None it means that no junction has beed detected within the sensor range
    """
    inattention_time: float = 0
    """
    Accumulates the amount of time (seconds) in which the driver is 
    not paying attention to the road
    """
    attention_time: float = 0
    """
    Accumulates the amount of time (seconds) in which the driver is 
    paying attention to the road
    """

    @property
    def speed_kmh(self) -> float:
        return self.speed.length() * 3.6

    vehicle: Vehicle
    vehicle_ackermann_control: VehicleAckermannControl | None = None
    """
    Applied a the end of each step only if not None
    """
    vehicle_control: VehicleControl = VehicleControl()
    """
    The Vehicle control to be applied at the end of each step
    Can be overridden by setting vehicle_ackermann_control != None
    """
    pygame_io: PygameIO
    manual_control: PygameVehicleControl
    dashboard_buttons: PygameDashboardButtons = PygameDashboardButtons()
    pygame_events: list[pygame.event.Event] = []
    wake_up_sound: pygame.mixer.Sound | None = None

    inattention_detector: InattentionDetector
    """
    Detector used to spot inattentive behaviours in the driver.
    """
    obstacles_detector: SafePulloverChecker

    def __init__(
        self,
        pygame_io: PygameIO,
        vehicle: Vehicle,
        world: World,
        map: Map,
        traffic_manager: TrafficManager,
        params: VehicleParams,
        driver_camera_stream: CameraStream,
        front_radar: Sensor,
        wake_up_sound: pygame.mixer.Sound | None,
        logging_config: VehicleLoggingConfig | None,
    ):
        if logging_config is None:
            self.logging_config = VehicleLoggingConfig()
        else:
            self.logging_config = logging_config
        self.params = params
        self.world = world
        self.map = map
        self.topology = map.get_topology()
        self.traffic_manager = traffic_manager
        self.vehicle = vehicle
        self.pygame_io = pygame_io
        self.manual_control = PygameVehicleControl(vehicle)
        self.inattention_detector = InattentionDetector(
            driver_camera_stream, eye_threshold=0.23
        )
        offset = float(vehicle.bounding_box.extent.y)
        self.obstacles_detector = SafePulloverChecker(
            radar_sensor=front_radar,
            vehicle=vehicle,
            min_inliers=2,
            scanned_area_x_offset=offset,
            debug=True,
        )
        self.wake_up_sound = wake_up_sound


class PullOverSafety(StrEnum):
    SAFE = "SAFE"
    GOING_TOO_FAST = "GOING TOO FAST"
    OBSTACLE_DETECTED = "OBSTACLE DETECTED"
    JUNCTION_DETECTED = "JUNCTION DETECTED"
    MISSING_EM_LANE = "NO EMERGENCY LANE ON THE RIGHT"


class VehicleTimers(StrEnum):
    INATTENTION = auto()
    INATTENTION_CHECK = auto()
    WAKE_UP_SOUND_DELAY = auto()


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
        traffic_manager: TrafficManager,
        params: VehicleParams,
        driver_camera_stream: CameraStream,
        front_radar: Sensor,
        wake_up_sound: pygame.mixer.Sound | None = None,
        logging_config: VehicleLoggingConfig | None = None,
    ):
        super().__init__(
            [WrapperS()],
            VehicleData(
                pygame_io=pygame_io,
                vehicle=vehicle,
                world=world,
                map=map,
                traffic_manager=traffic_manager,
                params=params,
                driver_camera_stream=driver_camera_stream,
                front_radar=front_radar,
                wake_up_sound=wake_up_sound,
                logging_config=logging_config,
            ),
            logging_config=logging_config,
        )

    @override
    def step(self, dt: float) -> bool:
        result = super().step(dt)
        if self._vehicle_logging_config().log_main_vehicle_controls:
            control = self._data.vehicle.get_control()
            acc = self._data.vehicle.get_acceleration()
            acc_len = acc.length()
            acc_len = (
                acc_len if acc.dot(self._data.vehicle.get_velocity()) >= 0 else -acc_len
            )
            self._vehicle_log("accel:", acc_len, "m/s^2")
            self._vehicle_log("speed:", self._data.speed_kmh, "km/h")
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
    def on_entry(self, data: VehicleData, ctx: VehicleContext):
        pygame.mixer.init()

    @override
    def on_early_do(self, data: VehicleData, ctx: VehicleContext):
        data.speed = data.vehicle.get_velocity()
        data.vehicle_control = VehicleControl()
        data.vehicle_ackermann_control = None
        data.pygame_events = data.pygame_io.update()
        data.manual_control.update(ctx.dt, data.pygame_events)
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

    @override
    def on_exit(self, data: VehicleData, ctx: VehicleContext):
        light_state = data.vehicle.get_light_state()
        data.vehicle.set_light_state(
            VehicleLightState(
                light_state
                & ~(VehicleLightState.RightBlinker | VehicleLightState.LeftBlinker)
            )
        )

        if data.wake_up_sound is not None:
            data.wake_up_sound.stop()


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
        data.traffic_manager.auto_lane_change(data.vehicle, False)  # pyright: ignore[reportUnknownMemberType]
        data.traffic_manager.set_desired_speed(
            data.vehicle, data.params.cruise_target_speed_kmh
        )
        data.vehicle.set_autopilot(True, data.traffic_manager.get_port())

    @override
    def on_do(self, data: VehicleData, ctx: VehicleContext):
        if data.inattention_detector.detect():
            data.inattention_time += ctx.dt
        else:
            data.attention_time += ctx.dt

    @override
    def on_exit(self, data: VehicleData, ctx: VehicleContext):
        data.vehicle.set_autopilot(False, data.traffic_manager.get_port())
        # The following line fixes a carla bug https://github.com/carla-simulator/carla/issues/7626
        # It appears that applying an empty VehicleControl will not do anything and
        # so we need to apply a vehicle control with some field set to a meaningless value
        data.vehicle.apply_control(VehicleControl(throttle=0.01))


def _reset_inattention_check_timer_and_accumulators(
    data: VehicleData, ctx: VehicleContext
):
    ctx.timer(VehicleTimers.INATTENTION_CHECK).reset(
        data.params.inattention_check_interval
    )
    data.inattention_time = 0
    data.attention_time = 0


def _inattention_detected(data: VehicleData) -> bool:
    tot = data.inattention_time + data.attention_time
    return data.attention_time < tot * data.params.attention_time_percentage


class NoInattentionDetectedS(VehicleState):
    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=InattentionDetectedS(),
                condition=lambda data, ctx: ctx.timer(
                    VehicleTimers.INATTENTION_CHECK
                ).is_elapsed()
                and _inattention_detected(data),
            )
        ]

    @override
    def actions(self) -> list[VehicleStateAction]:
        return [
            VehicleStateAction(
                condition=lambda data, ctx: ctx.timer(
                    VehicleTimers.INATTENTION_CHECK
                ).is_elapsed(),
                action=_reset_inattention_check_timer_and_accumulators,
            ),
        ]

    @override
    def on_entry(self, data: VehicleData, ctx: VehicleContext):
        _reset_inattention_check_timer_and_accumulators(data, ctx)


class InattentionDetectedS(VehicleState):
    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=NoInattentionDetectedS(),
                condition=lambda data, ctx: ctx.timer(
                    VehicleTimers.INATTENTION_CHECK
                ).is_elapsed()
                and not _inattention_detected(data),
                action=lambda data, ctx: data.wake_up_sound.stop()
                if data.wake_up_sound is not None
                else None,
            ),
            VehicleTransition(
                to=PullOverPreparationS(),
                condition=lambda data, ctx: ctx.timer(
                    VehicleTimers.INATTENTION
                ).is_elapsed(),
            ),
        ]

    @override
    def actions(self) -> list[VehicleStateAction]:
        return [
            VehicleStateAction(
                condition=lambda data, ctx: ctx.timer(
                    VehicleTimers.INATTENTION_CHECK
                ).is_elapsed(),
                action=_reset_inattention_check_timer_and_accumulators,
            ),
            VehicleStateAction(
                condition=lambda data, ctx: ctx.timer(
                    VehicleTimers.WAKE_UP_SOUND_DELAY
                ).is_elapsed(),
                action=lambda data, ctx: self._activate_wake_up_sound(data, ctx),
            ),
        ]

    def _activate_wake_up_sound(self, data: VehicleData, ctx: VehicleContext):
        if data.wake_up_sound is not None:
            _ = data.wake_up_sound.play(loops=-1)
        # Just a hack to not trigger again on every step
        ctx.timer(VehicleTimers.WAKE_UP_SOUND_DELAY).reset(math.inf)

    @override
    def on_entry(self, data: VehicleData, ctx: VehicleContext):
        ctx.timer(VehicleTimers.WAKE_UP_SOUND_DELAY).reset(
            data.params.wake_up_sound_delay
        )
        ctx.timer(VehicleTimers.INATTENTION).reset(18)
        _reset_inattention_check_timer_and_accumulators(data, ctx)


def _pull_over_is_safe(data: VehicleData) -> PullOverSafety:
    """
    Computes whether pulling over is safe by taking into account:
    - Eventual obstacles in the emergency lane
    - Ability to stop the vehicle before any junction
    """
    max_stop_dist = _max_stopping_distance(data)
    max_stop_dist = max(max_stop_dist, data.params.min_pull_over_space)
    scan_width = (
        data.vehicle.bounding_box.extent.y * 2 * 1.65
        - _signed_lateral_distance(
            data.vehicle.get_location(), _curr_waypoint(data).transform
        )
    )
    if max_stop_dist > data.params.sensors_max_range:
        return PullOverSafety.GOING_TOO_FAST
    if not data.obstacles_detector.is_pullover_safe(
        max_stop_dist + 10,
        scan_width,
        data.vehicle.get_wheel_steer_angle(VehicleWheelLocation.FR_Wheel),
    ):
        return PullOverSafety.OBSTACLE_DETECTED
    if not _right_lane_is_shoulder(data):
        return PullOverSafety.MISSING_EM_LANE
    if (
        data.first_junction_distance is not None
        and data.first_junction_distance <= max_stop_dist
    ):
        return PullOverSafety.JUNCTION_DETECTED
    return PullOverSafety.SAFE


def _waypoints_roughly_same_direction(w1: Waypoint, w2: Waypoint) -> bool:
    return w1.transform.get_forward_vector().dot(w2.transform.get_forward_vector()) > 0


def _waypoints_same_road(w1: Waypoint, w2: Waypoint) -> bool:
    return w1.road_id == w2.road_id


def _right_lane_is_shoulder(data: VehicleData) -> bool:
    right_lane = _curr_waypoint(data).get_right_lane()
    return right_lane is not None and right_lane.lane_type == LaneType.Shoulder


def _curr_waypoint(data: VehicleData) -> Waypoint:
    # We use the target waypoint as getting a waypoint under the vehicle position
    # may return a waypoint that is part of another overlapping lane
    next_action = cast(
        list[Waypoint | None], data.traffic_manager.get_next_action(data.vehicle)
    )
    if next_action.__len__() > 0 and next_action[1] is not None:
        target_waypoint = next_action[1]
        # Here we find a waypoint that is in the exact position of the car but on the correct lane
        distance_from_car = target_waypoint.transform.location.distance(
            data.vehicle.get_location()
        )
        return next(
            filter(
                lambda w: w.lane_id == target_waypoint.lane_id,
                target_waypoint.previous(distance_from_car),
            ),
            target_waypoint,  # Just to not crash if for some reason preconditions are not met
        )
    else:
        # In case there is no next action whe use the current location waypoint
        return data.map.get_waypoint(data.vehicle.get_location())


def _first_junction_detected_distance(data: VehicleData) -> float | None:
    # Here we compute a waypoint every meter ahead of the vehicle to later check if it is part of a junction
    waypoints = [_curr_waypoint(data)]
    # +1 because curr_waypoint is counted
    sensors_range = int(data.params.sensors_max_range) + 1
    while waypoints.__len__() < sensors_range:
        last_w = waypoints[waypoints.__len__() - 1]
        waypoints.extend(last_w.next_until_lane_end(1))
        last_w = waypoints[waypoints.__len__() - 1]
        first_of_next_section = next(
            filter(
                lambda tuple: last_w.road_id == tuple[0].road_id
                and last_w.section_id == tuple[0].section_id
                and last_w.lane_id == tuple[1].lane_id,
                data.topology,
            ),
            None,
        )
        if first_of_next_section is None:
            # Some roads intersections are connected in strange ways and i was unable
            # to reliably find the correct lane continuation, in which case we assume
            # there may be a junction (for safety reasons)
            return 0
        waypoints[waypoints.__len__() - 1] = first_of_next_section[1]

    waypoints = waypoints[:sensors_range]
    for distance, w in enumerate(waypoints):
        # Junctions are road segments and as such they include both sides, so checking if
        # the waypoint is part of a junction is not enough because we don't care if the
        # actual exit or entry is on the other side of the road
        if w.is_junction:
            junction_waypoints = w.get_junction().get_waypoints(LaneType.Any)

            # Here we take just those waypoints that point to the same direction as the vehicle
            junction_waypoints = filter(
                lambda t: _waypoints_roughly_same_direction(t[0], w)
                and _waypoints_roughly_same_direction(t[1], w),
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
            if waypoint_in_different_road is not None:
                return distance
    return None


def _max_stopping_distance(data: VehicleData) -> float:
    """
    Computes the stopping distance considering the maximum pull over deceleration
    and taking as current speed the maximum between the actual current speed
    and the minimum pull over speed
    """
    speed = max(data.speed.length(), data.params.min_pull_over_speed_kmh / 3.6)
    return (speed**2) / (2 * abs(data.params.max_pull_over_acceleration))


class PullOverPreparationS(VehicleState):
    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=PullingOverS(),
                condition=lambda data, ctx: _pull_over_is_safe(data)
                == PullOverSafety.SAFE,
            )
        ]

    @override
    def on_entry(self, data: VehicleData, ctx: VehicleContext):
        light_state = data.vehicle.get_light_state()
        data.vehicle.set_light_state(
            VehicleLightState(light_state | VehicleLightState.RightBlinker)
        )
        data.traffic_manager.set_desired_speed(
            data.vehicle,
            min(data.params.max_pull_over_preparation_speed_kmh, data.speed_kmh),
        )
        lane_offset = (
            (_curr_waypoint(data).lane_width / 2) - data.vehicle.bounding_box.extent.y
        ) * 0.9
        data.traffic_manager.vehicle_lane_offset(data.vehicle, lane_offset)

    @override
    def on_do(self, data: VehicleData, ctx: VehicleContext):
        # We cache this value as it is heavy to compute an needs to be used multiple times
        data.first_junction_distance = _first_junction_detected_distance(data)

    @override
    def on_exit(self, data: VehicleData, ctx: VehicleContext):
        data.traffic_manager.vehicle_lane_offset(data.vehicle, 0)

    @override
    def actions(self) -> list[VehicleStateAction]:
        return [
            VehicleStateAction(
                condition=lambda data, ctx: _pull_over_is_safe(data)
                != PullOverSafety.SAFE,
                action=lambda data, ctx: data.world.debug.draw_string(
                    data.vehicle.get_location() + Location(z=2),
                    _pull_over_is_safe(data).__str__(),
                ),
            )
        ]


# ========== PULLING_OVER ==========


def _changeVehicleAckermannControl(
    control: VehicleAckermannControl,
    steer: float | None = None,
    steer_speed: float | None = None,
    speed: float | None = None,
    acceleration: float | None = None,
    jerk: float | None = None,
) -> VehicleAckermannControl:
    return VehicleAckermannControl(
        steer=steer if steer is not None else control.steer,
        steer_speed=steer_speed if steer_speed is not None else control.steer_speed,
        speed=speed if speed is not None else control.speed,
        acceleration=acceleration if acceleration is not None else control.acceleration,
        jerk=jerk if jerk is not None else control.jerk,
    )


class PullingOverS(VehicleState):
    @override
    def children(self) -> list[VehicleState]:
        return [EmergencyLaneNotReachedS(), EmergencyLaneReachedS()]

    @override
    def on_entry(self, data: VehicleData, ctx: VehicleContext):
        max_pull_over_speed = (
            min(data.params.max_pull_over_preparation_speed_kmh, data.speed_kmh) / 3.6
        )
        pull_over_distance = (
            data.first_junction_distance
            if data.first_junction_distance is not None
            else _max_stopping_distance(data)
        )
        # We compute this value that is used in _keep_target_speed
        data.pull_over_acceleration = -(max_pull_over_speed**2) / (
            2 * pull_over_distance
        )

    @override
    def on_do(self, data: VehicleData, ctx: VehicleContext):
        # Here we use a motor schemas / potential fields approach to achieve
        # smooth and robust pull over manuver.
        # Two potential fields will influence the ego vehicle:
        # - A transversal field which pushes the vehicle to the emergency lane (pull_over_field)
        #   with constant magnitude
        # - A transversal field which pushes the vehicle away from the road margin (counter_field)
        #   with magnitude inversely proportional to the distance from the road margin (1/d)
        # Each field has an associated coefficient which tuning is fundamental as it determines
        # where the vehicle will stabilize (see VehicleParams)

        # In order to avoid excessive oscillating behavior the fields forces are evaluated
        # a bit ahead of the vehicle
        vehicle_front = _vehicle_front(data)
        vehicle_front = Location(
            vehicle_front + data.vehicle.get_transform().get_forward_vector() * 1
        )
        # Since it is of maximum importance not to exceed the emergency lane margin
        # the fields are evaluated on the "front right corner" of the vehicle
        vehicle_front = Location(
            vehicle_front
            + data.vehicle.get_transform().get_right_vector()
            * data.vehicle.bounding_box.extent.y
        )
        lane_w = cast(
            Waypoint, data.map.get_waypoint(vehicle_front, lane_type=LaneType.Shoulder)
        )

        signed_lateral_distance = _signed_lateral_distance(
            vehicle_front, lane_w.transform
        )
        pull_over_field = data.params.pull_over_potential_field_coeff
        distance_from_right_margin = signed_lateral_distance - lane_w.lane_width / 2
        counter_field = (
            1
            / distance_from_right_margin
            * data.params.road_margin_repulsive_potential_field_coeff
        )

        # The steer must be adjusted with respect to the vehicle speed as a slower vehicle
        # needs more steer in order to move laterally the same way a faster vehicle would do
        # The coefficient should be roughly between 0 and 1 given the maximum pull over speed
        speed_coeff = data.speed_kmh / data.params.max_pull_over_preparation_speed_kmh

        data.vehicle_ackermann_control = VehicleAckermannControl(
            steer_speed=0,
            steer=_steer_to_radians(
                data,
                (pull_over_field + counter_field) * (1 - speed_coeff),
            ),
        )


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
    waypoint = data.map.get_waypoint(
        data.vehicle.get_location(), lane_type=LaneType.Any
    )
    return waypoint.lane_type == LaneType.Shoulder


def _clamp(a: float, min_: float, max_: float) -> float:
    return min(max(a, min_), max_)


def _steer_to_radians(data: VehicleData, steer: float) -> float:
    max_steer_angle_deg = data.vehicle.get_physics_control().wheels[0].max_steer_angle
    return math.radians(max_steer_angle_deg) * _clamp(steer, -1, 1)


def _keep_target_speed(data: VehicleData, speed_kmh: float):
    """
    Keeps the target speed, if that requires to decelerate it will not exceed the
    maximum deceleration set in VehicleParams
    """
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
        acc = data.pull_over_acceleration
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
    if data.vehicle_ackermann_control is not None:
        data.vehicle_ackermann_control = _changeVehicleAckermannControl(
            data.vehicle_ackermann_control, acceleration=acc, speed=speed_ms
        )
    else:
        raise Exception("Expected to be using ackermann vehicle control")


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


class EmergencyLaneReachedS(VehicleState):
    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=StoppedS(),
                condition=lambda data, ctx: data.speed_kmh <= 0.5,
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


# ========== STOPPED ==========


class StoppedS(VehicleState):
    @override
    def on_entry(self, data: VehicleData, ctx: VehicleContext):
        data.vehicle_control.hand_brake = True
        data.vehicle_control.gear = 0
        light_state = data.vehicle.get_light_state()
        data.vehicle.set_light_state(
            VehicleLightState(
                light_state
                | VehicleLightState.LeftBlinker
                | VehicleLightState.RightBlinker
            )
        )
