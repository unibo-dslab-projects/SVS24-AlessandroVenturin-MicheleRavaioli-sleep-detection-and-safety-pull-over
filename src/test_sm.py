from time import sleep
from typing import override
from enum import StrEnum, auto
from sync_state_machine import SyncStateMachine

class State(StrEnum):
    MANUAL = auto()
    LANE_KEEPING = auto()
    PULLING_OVER = auto()

class Vehcle(SyncStateMachine[State]):
    speed: float = 0

    def __init__(self):
        super().__init__(State.MANUAL)

    @override
    def on_do(self, dt: float) -> None:
        match self.state():
            case State.MANUAL:
                self.speed = self.speed + 1 * dt
            case State.LANE_KEEPING:
                pass
            case State.PULLING_OVER:
                self.speed = self.speed - 1 * dt
        print(self.speed)
    
    @override
    def next_state(self, dt: float) -> State | None:
        return super().next_state(dt)

if __name__ == "__main__":
    vehicle = Vehcle()
    while True:
        sleep(0.1)
        vehicle.step(0.1)

