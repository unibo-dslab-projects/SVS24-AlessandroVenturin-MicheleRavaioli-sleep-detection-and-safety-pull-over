from abc import ABC
from enum import StrEnum, auto
from typing import final, override


class SyncStateMachine[State](ABC):
    __state: State
    __is_transitioning: bool = False

    @final
    def state(self) -> State:
        if self.__is_transitioning:
            print(
                "WARNING: you tried to access the current state during a transition. During a transition there's no notion of state you should instead check previous/next state"
            )
        return self.__state

    def on_entry(self, dt: float) -> None:  # pyright: ignore[reportUnusedParameter]
        return

    def on_do(self, dt: float) -> None:  # pyright: ignore[reportUnusedParameter]
        return

    def on_exit(self, dt: float) -> None:  # pyright: ignore[reportUnusedParameter]
        return

    def next_state(self, dt: float) -> State | None:  # pyright: ignore[reportUnusedParameter]
        return

    def on_state_transition(
        self,
        dt: float,  # pyright: ignore[reportUnusedParameter]
        prev_state: State,  # pyright: ignore[reportUnusedParameter]
        next_state: State,  # pyright: ignore[reportUnusedParameter]
    ) -> None:
        return

    @final
    def step(self, dt: float) -> None:
        next_state = self.next_state(dt)
        if next_state is not None:
            self.on_exit(dt)
            self.__is_transitioning = True
            self.on_state_transition(dt, self.state(), next_state)
            self.__is_transitioning = False
            self.on_entry(dt)
        self.on_do(dt)

    def __init__(self, initial_state: State):
        self.__state = initial_state
        self.on_entry(dt=0)

