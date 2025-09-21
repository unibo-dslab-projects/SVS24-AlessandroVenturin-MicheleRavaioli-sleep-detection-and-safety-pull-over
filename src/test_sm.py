from time import sleep
from typing import override
from enum import StrEnum, auto
from sync_state_machine import SyncStateMachine

class State(StrEnum):
    MANUAL = auto()
    LANE_KEEPING = auto()
    PULLING_OVER = auto()
    STOPPED = auto()

class Vehcle(SyncStateMachine[State]):
    speed: float = 0

    def __init__(self):
        super().__init__(State.MANUAL)

    @override
    def on_do(self, dt: float) -> None:
        match self.state():
            case State.MANUAL:
                self.speed = self.speed + 10 * dt
            case State.LANE_KEEPING:
                pass
            case State.PULLING_OVER:
                self.speed = self.speed - 10 * dt
            case State.STOPPED:
                pass
        print(self.speed)
    
    @override
    def next_state(self, dt: float) -> State | None:
        print(self.state())
        match self.state():
            case State.MANUAL:
                if self.speed > 50:
                    print("sos")
                    return State.LANE_KEEPING
            case State.LANE_KEEPING:
                pass
            case State.PULLING_OVER:
                if self.speed <= 0:
                    return State.STOPPED
            case State.STOPPED:
                pass


if __name__ == "__main__":
    vehicle = Vehcle()
    while True:
        sleep(0.1)
        vehicle.step(0.1)

