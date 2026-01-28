"""
Robot State Machine

Controls the high-level behavior of the robot.
Ensures safe transitions between operating modes.

States:
  IDLE       → Robot stopped, waiting for command
  MAPPING    → Building a map (manual or autonomous)
  LOCALIZING → Finding position on existing map
  NAVIGATING → Autonomous navigation to goal
  PAUSED     → Temporarily stopped (obstacle or user)
  EMERGENCY  → Emergency stop, requires manual reset
"""

import time
from enum import Enum, auto
from typing import Optional, Callable, Dict, List
from dataclasses import dataclass


class RobotState(Enum):
    """Robot operating states."""
    IDLE = auto()
    MAPPING = auto()
    LOCALIZING = auto()
    NAVIGATING = auto()
    PAUSED = auto()
    EMERGENCY = auto()


class StateEvent(Enum):
    """Events that trigger state transitions."""
    START_MAPPING = auto()
    START_NAVIGATION = auto()
    GOAL_REACHED = auto()
    PAUSE = auto()
    RESUME = auto()
    EMERGENCY_STOP = auto()
    RESET = auto()
    MAP_SAVED = auto()
    LOCALIZED = auto()
    NAVIGATION_FAILED = auto()
    OBSTACLE_BLOCKED = auto()


@dataclass
class StateTransition:
    """A state transition rule."""
    from_state: RobotState
    event: StateEvent
    to_state: RobotState
    condition: Optional[Callable[[], bool]] = None  # Optional guard


class StateMachine:
    """
    Finite state machine for robot behavior control.

    Valid transitions:
        IDLE ──START_MAPPING──► MAPPING
        IDLE ──START_NAV──────► LOCALIZING
        MAPPING ──MAP_SAVED──► IDLE
        LOCALIZING ──LOCALIZED──► NAVIGATING
        NAVIGATING ──GOAL_REACHED──► IDLE
        NAVIGATING ──OBSTACLE_BLOCKED──► PAUSED
        PAUSED ──RESUME──► NAVIGATING
        ANY ──EMERGENCY──► EMERGENCY
        EMERGENCY ──RESET──► IDLE

    Usage:
        sm = StateMachine()

        sm.on_enter(RobotState.NAVIGATING, start_navigation)
        sm.on_exit(RobotState.NAVIGATING, stop_motors)

        sm.handle_event(StateEvent.START_NAVIGATION)
        print(sm.state)  # RobotState.LOCALIZING
    """

    # Define all valid transitions
    TRANSITIONS = [
        # From IDLE
        StateTransition(RobotState.IDLE, StateEvent.START_MAPPING, RobotState.MAPPING),
        StateTransition(RobotState.IDLE, StateEvent.START_NAVIGATION, RobotState.LOCALIZING),

        # From MAPPING
        StateTransition(RobotState.MAPPING, StateEvent.MAP_SAVED, RobotState.IDLE),
        StateTransition(RobotState.MAPPING, StateEvent.PAUSE, RobotState.PAUSED),

        # From LOCALIZING
        StateTransition(RobotState.LOCALIZING, StateEvent.LOCALIZED, RobotState.NAVIGATING),
        StateTransition(RobotState.LOCALIZING, StateEvent.NAVIGATION_FAILED, RobotState.IDLE),

        # From NAVIGATING
        StateTransition(RobotState.NAVIGATING, StateEvent.GOAL_REACHED, RobotState.IDLE),
        StateTransition(RobotState.NAVIGATING, StateEvent.PAUSE, RobotState.PAUSED),
        StateTransition(RobotState.NAVIGATING, StateEvent.OBSTACLE_BLOCKED, RobotState.PAUSED),
        StateTransition(RobotState.NAVIGATING, StateEvent.NAVIGATION_FAILED, RobotState.IDLE),

        # From PAUSED
        StateTransition(RobotState.PAUSED, StateEvent.RESUME, RobotState.NAVIGATING),
        StateTransition(RobotState.PAUSED, StateEvent.RESET, RobotState.IDLE),

        # EMERGENCY (from any state)
        StateTransition(RobotState.IDLE, StateEvent.EMERGENCY_STOP, RobotState.EMERGENCY),
        StateTransition(RobotState.MAPPING, StateEvent.EMERGENCY_STOP, RobotState.EMERGENCY),
        StateTransition(RobotState.LOCALIZING, StateEvent.EMERGENCY_STOP, RobotState.EMERGENCY),
        StateTransition(RobotState.NAVIGATING, StateEvent.EMERGENCY_STOP, RobotState.EMERGENCY),
        StateTransition(RobotState.PAUSED, StateEvent.EMERGENCY_STOP, RobotState.EMERGENCY),

        # From EMERGENCY (only RESET)
        StateTransition(RobotState.EMERGENCY, StateEvent.RESET, RobotState.IDLE),
    ]

    def __init__(self):
        self._state = RobotState.IDLE
        self._previous_state: Optional[RobotState] = None
        self._state_start_time = time.time()

        # Callbacks
        self._on_enter: Dict[RobotState, List[Callable]] = {s: [] for s in RobotState}
        self._on_exit: Dict[RobotState, List[Callable]] = {s: [] for s in RobotState}
        self._on_transition: List[Callable[[RobotState, StateEvent, RobotState], None]] = []

        # Build transition lookup
        self._transition_map: Dict = {}
        for t in self.TRANSITIONS:
            key = (t.from_state, t.event)
            self._transition_map[key] = t

    @property
    def state(self) -> RobotState:
        """Current state."""
        return self._state

    @property
    def previous_state(self) -> Optional[RobotState]:
        """Previous state."""
        return self._previous_state

    @property
    def time_in_state(self) -> float:
        """Time spent in current state (seconds)."""
        return time.time() - self._state_start_time

    @property
    def is_moving(self) -> bool:
        """True if robot should be moving."""
        return self._state in (RobotState.MAPPING, RobotState.NAVIGATING)

    @property
    def is_safe_to_move(self) -> bool:
        """True if it's safe to move."""
        return self._state not in (RobotState.EMERGENCY, RobotState.IDLE)

    def handle_event(self, event: StateEvent) -> bool:
        """
        Handle a state event.

        Args:
            event: The event to handle

        Returns:
            True if transition occurred, False if event was ignored
        """
        key = (self._state, event)
        transition = self._transition_map.get(key)

        if transition is None:
            return False

        # Check guard condition
        if transition.condition and not transition.condition():
            return False

        # Execute transition
        old_state = self._state
        new_state = transition.to_state

        # Exit callbacks
        for callback in self._on_exit[old_state]:
            callback()

        # Update state
        self._previous_state = old_state
        self._state = new_state
        self._state_start_time = time.time()

        # Transition callbacks
        for callback in self._on_transition:
            callback(old_state, event, new_state)

        # Enter callbacks
        for callback in self._on_enter[new_state]:
            callback()

        return True

    def on_enter(self, state: RobotState, callback: Callable):
        """Register callback for entering a state."""
        self._on_enter[state].append(callback)

    def on_exit(self, state: RobotState, callback: Callable):
        """Register callback for exiting a state."""
        self._on_exit[state].append(callback)

    def on_transition(self, callback: Callable[[RobotState, StateEvent, RobotState], None]):
        """Register callback for any transition."""
        self._on_transition.append(callback)

    def get_status(self) -> dict:
        """Get state machine status."""
        return {
            "state": self._state.name,
            "previous": self._previous_state.name if self._previous_state else "N/A",
            "time_in_state": f"{self.time_in_state:.1f}s",
            "is_moving": self.is_moving,
        }