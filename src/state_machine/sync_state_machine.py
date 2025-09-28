from __future__ import annotations
from abc import ABC
from collections.abc import Iterable, Sequence
from typing import Protocol, cast, override


class Condition[Data, Timers](Protocol):
    """
    A function that evaluates to a boolean value
    """

    def __call__(self, data: Data, ctx: Context[Timers]) -> bool: ...


class Action[Data, Timers](Protocol):
    """
    A function that modifies the data when executed
    """

    def __call__(self, data: Data, ctx: Context[Timers]): ...


class Transition[Data, Timers]:
    """
    A transition to another state, it can be guarded by a condition and it can also perform an action once triggered
    """

    condition: Condition[Data, Timers]

    next_state: State[Data, Timers]

    action: Action[Data, Timers]

    def __init__(
        self,
        to: State[Data, Timers],
        condition: Condition[Data, Timers] = lambda data, ctx: True,
        action: Action[Data, Timers] = lambda data, ctx: None,
    ):
        """
        Args:
            to: the state to transition to
            condition: condition what when evaluated to true will trigger the transition
            action: action to be executed once the transition is triggered
        """
        self.next_state = to
        self.condition = condition
        self.action = action


class StateAction[Data, Timers]:
    """
    An action that is performed when its condition evaluates to true (without changing the active state of the machine)
    """

    condition: Condition[Data, Timers]

    action: Action[Data, Timers]

    def __init__(
        self,
        condition: Condition[Data, Timers] = lambda data, ctx: True,
        action: Action[Data, Timers] = lambda data, ctx: None,
    ):
        """
        Args:
            condition: condition what when evaluated to true will trigger the action
            action: action to be executed once the condition is satisfied
        """
        self.condition = condition
        self.action = action


class State[Data, Timers](ABC):
    """
    Description of a state machine state.
    This is just a description of a state and therefore instances should not
    hold any data/state (to do so use the Data type parameter)
    """

    def children(self) -> Sequence[State[Data, Timers]]:
        """
        List of children states, the first one will be the child to enter when entering this state
        """
        return []

    def actions(self) -> Sequence[StateAction[Data, Timers]]:
        """
        List of actions that this state can perform.
        They will be executed in the same order they have beed defined.
        """
        return []

    def on_entry(self, data: Data, ctx: Context[Timers]):  # pyright: ignore[reportUnusedParameter]
        """
        Callback that is executed every time this state becomes active
        """
        ...

    def on_early_do(self, data: Data, ctx: Context[Timers]):  # pyright: ignore[reportUnusedParameter]
        """
        Callback that is executed on every step of the state machine while this state is active (before on_do)
        """
        ...

        ...
    def on_do(self, data: Data, ctx: Context[Timers]):  # pyright: ignore[reportUnusedParameter]
        """
        Callback that is executed on every step of the state machine while this state is active
        """
        ...

    def on_late_do(self, data: Data, ctx: Context[Timers]):  # pyright: ignore[reportUnusedParameter]
        """
        Callback that is executed on every step of the state machine while this state is active (after on_do)
        """
        ...

    def on_exit(self, data: Data, ctx: Context[Timers]):  # pyright: ignore[reportUnusedParameter]
        """
        Callback that is executed every time this state becomes inactive
        """
        ...

    def transitions(self) -> Sequence[Transition[Data, Timers]]:
        """
        The transitions that originates from this state
        """
        return list()

    def is_exit_state(self) -> bool:
        """
        An exit state is a state that when reached will result in the state machine terminating.
        on_early_do, on_do, on_late_do, on_exit callbacks are meaningless for this state and will
        not be called.
        transitions and children methods should not be overrided and they will be ignored otherwise
        """
        return False

    # Equality just checks the type
    @override
    def __eq__(self, value: object, /) -> bool:
        if not isinstance(value, State):
            return NotImplemented
        return type(self) is type(cast(State[Data, Timers], value))

    @override
    def __hash__(self) -> int:
        return hash(type(self))

def _lowest_entry_child[D, T](s: State[D, T]) -> State[D, T]:
    """
    Given a state it follows it's entry child down to the innermost
    """
    entry_child = s.children()[0] if s.children().__len__() > 0 else None
    if entry_child is None:
        return s
    else:
        return _lowest_entry_child(entry_child)


class Context[Timers]:
    """
    Context of a state machine, it carries the delta time between steps and all the defined timers
    """

    _dt: float
    _timers: dict[Timers, Timer] = {}

    @property
    def dt(self):
        """
        Amount of time (seconds) elapsed between the previous step and the current one
        """
        return self._dt

    def timer(self, timer: Timers) -> Timer:
        """
        Returns the requested timer
        """
        t = self._timers.get(timer)
        match t:
            case None:
                self._timers[timer] = Timer(0)
                return self._timers[timer]
            case t:
                return t

    def _step(self, dt: float):
        """
        Sets the delta time and updates all timers
        """
        self._dt = dt
        for t in self._timers.values():
            t.step(self._dt)

    def __init__(self, dt: float):
        self._dt = dt


class SyncStateMachine[Data, Timers](ABC):
    """
    Type parametes:
        Data: The type representing all the data that is part of the state machine
        Timers: The type of the keys used to reference timers (suggestion: use an enum)

    The state machine lifecycle is the following:
        1. The state machine (when initialized) starts in the initial state with dt=0 and
           triggers on_entry callbacks
        2. Once for every step:
            2a. Evaluates all the transition conditions of the active state (stopping at the
                first evaluating to True)
            2b. Performs the transition if one is triggered:
                2ba. Triggers the on_exit callbacks for the exiting state
                2bb. Executes the transition action (if defined)
                2bc. Sets the current state as the one defined by the transition
                2bd. Triggers the on_entry callbacks for the entering state
            2c. For each ancestor of the active state and for the active state itself
                2db. Triggers on_early_do callbacks
            2d. For each ancestor of the active state and for the active state itself
                2da. Check and execute state actions
                2db. Triggers on_do callbacks
            2e. For each ancestor of the active state and for the active state itself
                2ea. Triggers on_late_do callbacks

    Concepts to take into consideration:
        1. When entering a state the machine will call on_entry on every one of its ancestors
           from the outermost to the innermost and then on the state itself.
           When transitioning between two states the on_entry is called starting by the lowest
           common ancestor.
        2. During each step the on_early_do/on_do/on_late_do callback and any eventual states
           action will be called for each of the active state ancestors from the outermost to
           the innermost an then for the active state itself.
           Example:
               State A is an ancestor of state B
               1. trigger on_early_do for state A
               2. trigger on_early_do for state B
               3. execute actions of state A
               4. trigger on_do for state A
               5. execute actions of state B
               6. trigger on_do for state B
               7. trigger on_late_do for state A
               8. trigger on_late_do for state B
        3. When exiting a state the machine will call on_exit on the state itself and then on
           every one of its ancestors from the innermost to the outermost.
           When transitioning between two state the on_exit is called up to the lowest common
           ancestor (excluded).
        4. State actions are executed in the order they were defined by the state
        5. on_early_do, on_do, on_late_do, on_exit callbacks are meaningless for the exit states
           and will not be called even if defined (as well as transitions and children methods)
    """

    _parents: dict[State[Data, Timers], State[Data, Timers]] = {}
    """
    This dictionary associates states to their parent.
    If a state does not have parent then it will not be present in the dictionary
    """
    _ancestors: dict[State[Data, Timers], list[State[Data, Timers]]] = {}
    """
    Dictionary associating states with a list of their ancestors going from the inside
    to the outside.
    States without ancestors are associated to an empty list, so that it is save to call
    _ancestors[key] and assume you'll never get None
    """
    _state: State[Data, Timers]
    _data: Data
    _context: Context[Timers]

    def _build_parents_dict(
        self,
        top_level_states: Iterable[State[Data, Timers]],
        acc: dict[State[Data, Timers], State[Data, Timers]],
    ):
        for p in top_level_states:
            for c in p.children():
                if acc.get(c) is None:
                    acc[c] = p
                    self._build_parents_dict(p.children(), acc)

    def _build_ancestors_dict(
        self,
        top_level_states: Iterable[State[Data, Timers]],
        acc: dict[State[Data, Timers], list[State[Data, Timers]]],
    ):
        def ancestors_rec(
            child: State[Data, Timers], acc: list[State[Data, Timers]]
        ) -> list[State[Data, Timers]]:
            """
            Recursively computes the ancestors of a state
            """
            match self._parents.get(child):
                case None:
                    return acc
                case parent:
                    return ancestors_rec(parent, acc + [parent])

        for s in top_level_states:
            if acc.get(s) is None:
                acc[s] = ancestors_rec(s, [])
                self._build_ancestors_dict(s.children(), acc)

    def _lowest_common_ancestor(
        self, s1: State[Data, Timers], s2: State[Data, Timers]
    ) -> State[Data, Timers] | None:
        """
        Computes the lowest common ancestor between two states
        """
        ancestors_2 = set(self._ancestors[s2])
        for p1 in self._ancestors[s1]:
            if ancestors_2.__contains__(p1):
                return p1


    def __init__(self, top_level_states: Sequence[State[Data, Timers]], data: Data):
        """
        Starts a state machine.
        Args:
            top_level_states: all the top level states (first one will be the entry one)
            data: the initial data
        """
        self._build_parents_dict(top_level_states, self._parents)
        self._build_ancestors_dict(top_level_states, self._ancestors)
        self._state = _lowest_entry_child(top_level_states[0])
        self._data = data
        self._context = Context(0)
        self._entry_states(
            list(reversed(self._ancestors[self._state])) + [self._state],
            self._context,
        )

    def _entry_states(self, states: list[State[Data, Timers]], ctx: Context[Timers]):
        """
        Enters (calling on_entry) all the given states in the same order they are provided
        """
        for s in states:
            s.on_entry(self._data, ctx)

    def step(self, dt: float)-> bool:
        """
        Advances the state machine by dt seconds.
        Returns:
            True -> if the state machine is not exited
            False -> if the state machine is exited
        """
        self._context._step(dt)  # pyright: ignore[reportPrivateUsage]
        if self._state.is_exit_state():
            print("WARNING: step was called on an already exited state machine")
        transition = next(
            (
                t
                for t in self._state.transitions()
                if t.condition(self._data, self._context)
            ),
            None,
        )
        if transition is not None and self._state.is_exit_state() is False:
            next_state = _lowest_entry_child(transition.next_state)
            lca = self._lowest_common_ancestor(self._state, next_state)

            # exit from state and from all ancestors up to the lowest common ancestor
            self._state.on_exit(self._data, self._context)
            for p in self._ancestors[self._state]:
                if lca is None or lca == p:
                    break
                else:
                    lca.on_exit(self._data, self._context)

            transition.action(self._data, self._context)
            self._state = next_state

            # entry into all ancestors down from the lowest common ancestor and then into state
            reversed_ancestors = list(reversed(self._ancestors[self._state]))
            if lca is not None:
                # removing lca and outer ancestor
                while reversed_ancestors.pop(0) != lca:
                    ...
            self._entry_states(reversed_ancestors + [self._state], self._context)

        if self._state.is_exit_state():
            return False
        else:
            reversed_ancestors_and_self = list(
                reversed(self._ancestors[self._state])
            ) + [self._state]

            # on_early_do for all ancestors down from the lowest common ancestor and then into state
            for p in reversed_ancestors_and_self:
                p.on_early_do(self._data, self._context)

            # on_do and state actions for all ancestors down from the lowest common ancestor and then into state
            for p in reversed_ancestors_and_self:
                for a in p.actions():
                    if a.condition(self._data, self._context):
                        a.action(self._data, self._context)
                p.on_do(self._data, self._context)

            # on_late_do for all ancestors down from the lowest common ancestor and then into state
            for p in reversed_ancestors_and_self:
                p.on_late_do(self._data, self._context)

            return True

class Timer:
    _time_set: float
    _time_left: float

    def __init__(self, time_set: float = 0):
        self._time_set = time_set
        self.reset(time_set)

    def reset(self, time_set: float | None = None):
        """
        Resets the timer to time_set or, if None is provided, to the previous time set
        """
        match time_set:
            case None:
                self._time_left = self._time_set
            case time_set:
                self._time_left = time_set

    def step(self, dt: float):
        """
        Advances the timer of dt seconds
        """
        if not self.is_elapsed():
            self._time_left = self._time_left - dt

    def is_elapsed(self) -> bool:
        """
        Evaluates to true if the timer has reached 0
        """
        return self._time_left <= 0
