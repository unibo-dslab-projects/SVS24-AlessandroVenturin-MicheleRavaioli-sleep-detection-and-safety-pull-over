from enum import StrEnum, auto
from typing import override

from carla import Vector3D, Vehicle, VehicleControl

from state_machine.sync_state_machine import (
    Context,
    State,
    StateAction,
    SyncStateMachine,
    Transition,
)


class VehicleData:
    enable_logging: bool
    speed: Vector3D = Vector3D()
    should_toggle_lane_keeping: bool = False
    should_enter_manual_driving: bool = False

    vehicle_actor: Vehicle
    vehicle_control: VehicleControl = VehicleControl()

    def __init__(self, vehicle_actor: Vehicle, enable_logging: bool = False):
        self.enable_logging = enable_logging
        self.vehicle_actor = vehicle_actor


class VehicleTimers(StrEnum):
    INATTENTION = auto()


type VehicleContext = Context[VehicleTimers]


class VehicleState(State[VehicleData, VehicleTimers]): ...


class VehicleStateAction(StateAction[VehicleData, VehicleTimers]): ...


class VehicleTransition(Transition[VehicleData, VehicleTimers]): ...


class VehicleStateMachine(SyncStateMachine[VehicleData, VehicleTimers]):
    def __init__(self, vehicle_actor: Vehicle, enable_logging: bool = False):
        super().__init__(
            [WrapperS()],
            VehicleData(vehicle_actor=vehicle_actor, enable_logging=enable_logging),
        )
        self._log_current_state()

    @override
    def step(self, dt: float):
        super().step(dt)
        self._log_current_state()

    def _log_current_state(self):
        print(f"current state: {self._state.__class__.__name__}")


class WrapperS(State[VehicleData, VehicleTimers]):
    @override
    def children(self) -> list[VehicleState]:
        return [ManualDrivingS(), LaneKeepingS(), PullingOverS(), StoppedS()]

    @override
    def on_early_do(self, data: VehicleData, ctx: VehicleContext):
        data.speed = data.vehicle_actor.get_velocity()
        data.vehicle_control = VehicleControl()

    @override
    def on_late_do(self, data: VehicleData, ctx: VehicleContext):
        data.vehicle_actor.apply_control(data.vehicle_control)


# ========== MANUAL_DRIVING ==========


class ManualDrivingS(VehicleState):
    @override
    def children(self) -> list[VehicleState]:
        return [LaneKeepingS()]

    @override
    def transitions(self) -> list[VehicleTransition]:
        return [
            VehicleTransition(
                to=LaneKeepingS(),
                condition=lambda data, ctx: data.should_toggle_lane_keeping,
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
                condition=lambda data, ctx: data.should_toggle_lane_keeping,
            )
        ]


def _inattention_detected(data: VehicleData) -> bool:
    # TODO
    return True


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
    return True


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
