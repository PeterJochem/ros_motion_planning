from typing import List, Tuple
import numpy as np
from .robot_state import RobotState
from .path import Path


class TimeScaler:
    """ Scales a Path through time to make a Trajectory. """

    def __init__(self, *args, **kwargs):
        """ Constructor. """

        ...


class NaiveTimeScaler(TimeScaler):
    """ Naively chooses an extremely slow trajectory which any robot should be able to achieve. """

    def __init__(self):
        """ Constructor. """

        super().__init__()

    def scale(self, path: Path) -> List[Tuple[RobotState, float]]:
        """ Scales this instance's path through time.

        Args:
            path: Path

        Returns:
            List[Tuple[RobotState, float]]
                The Trajectories underlying data.
        """

        times = np.linspace(0., len(path.robot_states), len(path.robot_states))
        return [(state, time) for state, time in zip(path.robot_states, times)]
