from __future__ import annotations
from abc import ABC
from typing import Protocol, cast, final, override


class Condition[Data](Protocol):
    def __call__(self, data: Data) -> bool: ...


class MakeNextState[Data](Protocol):
    def __call__(self) -> State[Data]: ...


class Action[Data](Protocol):
    def __call__(self, data: Data) -> Data: ...


class Transition[Data]:
    condition: Condition[Data]

    make_next_state: MakeNextState[Data]

    action: Action[Data]

    def __init__(
        self,
        to: MakeNextState[Data],
        condition: Condition[Data] = lambda data: True,
        action: Action[Data] = lambda data: data,
    ) -> None:
        self.make_next_state = to
        self.condition = condition
        self.action = action


class State[Data](ABC):
    def parent(self) -> State[Data] | None:
        return None

    def entry_child(self) -> State[Data] | None:
        return None

    def on_entry(self, dt: float, data: Data) -> Data:  # pyright: ignore[reportUnusedParameter]
        return data

    def on_do(self, dt: float, data: Data) -> Data:  # pyright: ignore[reportUnusedParameter]
        return data

    def on_exit(self, dt: float, data: Data) -> Data:  # pyright: ignore[reportUnusedParameter]
        return data

    def transitions(self) -> list[Transition[Data]]:
        return list()

    @final
    def ancestors(self) -> list[State[Data]]:
        return _ancestors_rec(self, [])

    # Equality just checks the type
    @override
    def __eq__(self, value: object, /) -> bool:
        if not isinstance(value, State):
            return NotImplemented
        return type(self) is type(cast(State[Data], value))

    @override
    def __hash__(self) -> int:
        return hash(type(self))


def _ancestors_rec[D](child: State[D], acc: list[State[D]]) -> list[State[D]]:
    match child.parent():
        case None:
            return acc
        case parent:
            return _ancestors_rec(parent, acc + [parent])


def _lowest_common_ancestor[D](s1: State[D], s2: State[D]) -> State[D] | None:
    ancestors_2 = set(s2.ancestors())
    for p1 in s1.ancestors():
        if ancestors_2.__contains__(p1):
            return p1


def _lowest_entry_child[D](s: State[D]) -> State[D]:
    entry_child = s.entry_child()
    if entry_child is None:
        return s
    else:
        return _lowest_entry_child(entry_child)

class SyncStateMachine[Data](ABC):
    __state: State[Data]
    __data: Data

    def __init__(self, state: State[Data], data: Data):
        self.__state = _lowest_entry_child(state)
        self.__data = data
        dt = 0
        self.__data = self.__entry_states(
            dt, list(reversed(self.__state.ancestors())) + [self.__state]
        )

    def __entry_states(self, dt: float, states: list[State[Data]]) -> Data:
        for s in states:
            self.__data = s.on_entry(dt, self.__data)
        return self.__data

    @final
    def step(self, dt: float) -> Data:
        transition = next(
            (t for t in self.__state.transitions() if t.condition(self.__data)), None
        )
        if transition is not None:
            next_state = transition.make_next_state()
            next_state = _lowest_entry_child(next_state)
            lca = _lowest_common_ancestor(self.__state, next_state)

            # exit from state and from all ancestors up to the lowest common ancestor
            self.__data = self.__state.on_exit(dt, self.__data)
            for p in self.__state.ancestors():
                if lca is None or lca == p:
                    break
                else:
                    self.__data = lca.on_exit(dt, self.__data)

            self.__data = transition.action(self.__data)
            self.__state = next_state

            # entry into all ancestors down from the lowest common ancestor and then into state
            reversed_ancestors = list(reversed(self.__state.ancestors()))
            if lca is not None:
                # removing lca and outer ancestor
                while reversed_ancestors.pop(0) != lca:
                    ...
            self.__data = self.__entry_states(dt, reversed_ancestors + [self.__state])

        # on_do for all ancestors down from the lowest common ancestor and then into state
        for p in list(reversed(self.__state.ancestors())):
            self.__data = p.on_do(dt, self.__data)
        self.__data = self.__state.on_do(dt, self.__data)
        return self.__data

class Timer:
    _time_set: float
    _time_left: float

    def __init__(self, time_set: float = 0):
        self._time_set = time_set
        self.reset(time_set)

    def reset(self, time_set: float | None = None):
        match time_set:
            case None:
                self._time_left = self._time_set
            case time_set:
                self._time_left = time_set

    def step(self, dt: float):
        if not self.is_elapsed():
            self._time_left = self._time_left - dt

    def is_elapsed(self) -> bool:
        return self._time_left <= 0
