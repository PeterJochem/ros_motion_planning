from abc import ABC, abstractmethod
from .trajectory import Trajectory
from .path import Path
from .robot_state import RobotState


class Planner(ABC):
    """ Base class which all other planner's inherit from."""

    def __init__(self, *args, **kwargs):
        """ Constructor. """
        
        ...  # pragma no cover

    @abstractmethod
    def plan_to_joint_state(self, *args, **kwargs) -> Trajectory:
        """ """

        ...  # pragma no cover

    @abstractmethod
    def plan_to_pose(self, *args, **kwargs) -> Trajectory:
        """ """
        
        ...  # pragma no cover

    @abstractmethod
    def scale(self, path: Path) -> Trajectory:
        """ Scales the provided path through time.

        Args:
            path: Path

        Returns:
            Trajectory
        """

        ...  # pragma no cover

    def is_request_legal(self) -> bool:
        """ Checks if this instance's request is legal. """

        return self.start_state.is_legal(self.robot) and self.goal_state.is_legal(self.robot)

    @property
    def robot_state_type(self) -> RobotState:
        """ Gets the subclass of RobotState that this planner uses. """

        if self.request.robot_state_type is None:
            raise RuntimeError()
        return self.request.robot_state_type
