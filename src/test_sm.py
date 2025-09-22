from time import sleep
from typing import override
from sync_state_machine import State, SyncStateMachine, Transition


class CarData:
    speed: float

    def __init__(self, speed: float):
        self.speed = speed


class ManualState(State[CarData]):
    @override
    def parent(self) -> State[CarData] | None:
        return CarTurnedOnState()

    @override
    def on_do(self, dt: float, data: CarData) -> CarData:
        data.speed = data.speed + 10 * dt
        return data

    @override
    def transitions(self) -> list[Transition[CarData]]:
        return [
            Transition[CarData](
                to=lambda: LaneKeepingState(),
                condition=lambda data: data.speed > 50,
            )
        ]


class LaneKeepingState(State[CarData]):
    @override
    def parent(self) -> State[CarData] | None:
        return CarTurnedOnState()

    @override
    def on_do(self, dt: float, data: CarData) -> CarData:
        data.speed = data.speed + 1 * dt
        return data

    @override
    def transitions(self) -> list[Transition[CarData]]:
        return [
            Transition[CarData](
                to=lambda: PullingOverState(),
                condition=lambda data: data.speed > 55,
                action=lambda data: (print("WARNING: pulling over"), data)[1],
            )
        ]


class PullingOverState(State[CarData]):
    @override
    def parent(self) -> State[CarData] | None:
        return CarTurnedOnState()

    @override
    def on_do(self, dt: float, data: CarData) -> CarData:
        data.speed = data.speed - 10 * dt
        return data

    @override
    def transitions(self) -> list[Transition[CarData]]:
        return [
            Transition[CarData](
                to=lambda: StoppedState(),
                condition=lambda data: data.speed <= 0,
            )
        ]


class StoppedState(State[CarData]):
    @override
    def parent(self) -> State[CarData] | None:
        return CarTurnedOnState()

    @override
    def on_entry(self, dt: float, data: CarData) -> CarData:
        data.speed = 0
        return data


class CarTurnedOnState(State[CarData]):
    @override
    def parent(self) -> State[CarData] | None:
        return IAmACarState()
    @override
    def on_do(self, dt: float, data: CarData) -> CarData:
        print(f"speed: {data.speed}")
        return data

    @override
    def on_entry(self, dt: float, data: CarData) -> CarData:
        print("entering: TurnedOnState")
        return data

    @override
    def on_exit(self, dt: float, data: CarData) -> CarData:
        print("exiting: TurnedOnState")
        return data

class IAmACarState(State[CarData]):
    @override
    def on_do(self, dt: float, data: CarData) -> CarData:
        print("i am a car")
        return data

    @override
    def on_entry(self, dt: float, data: CarData) -> CarData:
        print("entering: IAmACarState")
        return data

    @override
    def on_exit(self, dt: float, data: CarData) -> CarData:
        print("exiting: IAmACarState")
        return data

class VehicleSM(SyncStateMachine[CarData]):
    def __init__(self, data: CarData):
        super().__init__(ManualState(), data)


# TODO: timeout from stopped to car turned off
if __name__ == "__main__":
    vehicle = VehicleSM(CarData(speed=30))
    while True:
        sleep(0.1)
        print(vehicle.step(0.1).speed)
