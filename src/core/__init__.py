"""
Core infrastructure module.
- Configuration management
- State machine
- Logging
- Safety systems
"""

from .state_machine import StateMachine, RobotState, StateEvent
from .safety import SafetyMonitor, SafetyConfig, SafetyStatus