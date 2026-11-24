"""Scale-aware contact transfer; angular-first, power-consistent conventions."""

from .robot import Robot
from .symbolic import SymbolicRobot
from .optimization import SolverSettings
from .conic_backends import ConicSettings
from .configuration import TaskConfiguration,load_task_configuration

__all__ = ["Robot", "SymbolicRobot", "SolverSettings", "ConicSettings", "TaskConfiguration", "load_task_configuration"]
__version__ = "0.2.0a2"
