from enum import StrEnum, auto
from time import sleep
from typing import override
from state_machine.sync_state_machine import Context, State, StateAction, SyncStateMachine, Transition


class CarTimers(StrEnum):
    TURN_ON = auto()


class CarData:
    speed: float

    def __init__(self, speed: float):
        self.speed = speed


class CarState(State[CarData, CarTimers]): ...


class CarStateAction(StateAction[CarData, CarTimers]): ...


class CarTransition(Transition[CarData, CarTimers]): ...


type CarContext = Context[CarTimers]


class ManualState(CarState):
    @override
    def on_do(self, data: CarData, ctx: CarContext):
        data.speed = data.speed + 10 * ctx.dt

    @override
    def transitions(self) -> list[CarTransition]:
        return [
            CarTransition(
                to=LaneKeepingState(),
                condition=lambda data, ctx: data.speed > 50,
            )
        ]


class LaneKeepingState(CarState):
    @override
    def on_do(self, data: CarData, ctx: CarContext):
        data.speed = data.speed + 1 * ctx.dt

    @override
    def transitions(self) -> list[CarTransition]:
        return [
            CarTransition(
                to=PullingOverState(),
                condition=lambda data, ctx: data.speed > 55,
                action=lambda data, ctx: print("WARNING: pulling over"),
            )
        ]


class PullingOverState(CarState):
    @override
    def on_do(self, data: CarData, ctx: CarContext):
        data.speed = data.speed - 10 * ctx.dt

    @override
    def transitions(self) -> list[CarTransition]:
        return [
            CarTransition(
                to=StoppedState(),
                condition=lambda data, ctx: data.speed <= 0,
            )
        ]


class StoppedState(CarState):
    @override
    def on_entry(self, data: CarData, ctx: CarContext):
        print("entering: StoppedState")
        data.speed = 0


class CarTurnedOnState(CarState):
    @override
    def children(self) -> list[CarState]:
        return [
            ManualState(),
            LaneKeepingState(),
            PullingOverState(),
            StoppedState(),
        ]

    @override
    def on_do(self, data: CarData, ctx: CarContext):
        print(f"speed: {data.speed}")

    @override
    def on_entry(self, data: CarData, ctx: CarContext):
        print("entering: TurnedOnState")

    @override
    def on_exit(self, data: CarData, ctx: CarContext):
        print("exiting: TurnedOnState")


class CarTurnedOffState(CarState):
    @override
    def on_entry(self, data: CarData, ctx: CarContext):
        print("entering: TurnedOffState")
        ctx.timer(CarTimers.TURN_ON).reset(5)
        print("turn on car in 5 seconds")

    @override
    def on_exit(self, data: CarData, ctx: CarContext):
        print("exiting: TurnedOffState")

    @override
    def transitions(self) -> list[CarTransition]:
        return [
            CarTransition(
                to=CarTurnedOnState(),
                condition=lambda data, ctx: ctx.timer(CarTimers.TURN_ON).is_elapsed(),
            )
        ]


class IAmACarState(CarState):
    @override
    def children(self) -> list[CarState]:
        return [CarTurnedOffState(), CarTurnedOnState()]

    @override
    def on_entry(self, data: CarData, ctx: CarContext):
        print("entering: IAmACarState")

    @override
    def on_exit(self, data: CarData, ctx: CarContext):
        print("exiting: IAmACarState")

    @override
    def actions(self) -> list[CarStateAction]:
        return [
            CarStateAction(
                condition=lambda data, ctx: data.speed % 5 == 0,
                action=lambda data, ctx: print("speed is a multiple of 5"),
            )
        ]


class VehicleSM(SyncStateMachine[CarData, CarTimers]):
    def __init__(self, data: CarData):
        super().__init__([IAmACarState()], data)


if __name__ == "__main__":
    data = CarData(speed=30)
    vehicle = VehicleSM(data)
    should_continue = True
    while True:
        sleep(0.1)
        should_continue = vehicle.step(0.1)
        print(data.speed)
