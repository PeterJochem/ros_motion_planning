from abc import ABC, abstractmethod
from robot import Robot
from .offline_planner import OfflinePlanner
from .planning_request import PlanningRequest
from .path import Path
from .robot_state import RobotState, RobotStateCollection


class RRTBase(OfflinePlanner, ABC):
    """ Base class for Rapidly Exploring Random Tree variants. """

    def __init__(self, robot: Robot, request: PlanningRequest):
        """ Constructor.

        Args:
            robot: Robot
            request: PlanningRequest
        """

        super().__init__(robot, request)
        self.delta_magnitude = request.delta_magnitude
        self.stopping_distance = request.stopping_distance

    def is_request_legal(self, request: PlanningRequest) -> bool:
        """ """

        if not super().is_request_legal():
            return False
        elif not hasattr(request, 'delta_magnitude') or request.delta_magnitude is None:
            return False
        elif not hasattr(request, 'stopping_distance') or request.stopping_distance is None:
            return False

        return True

    def is_done(self, robot_states: RobotStateCollection) -> bool:
        """ Checks if the provided tree contains a solution.

        Args:
            robot_states: RobotStateCollection

        Returns:
            bool
        """

        return robot_states.distance_to_nearest(self.goal_state) < self.stopping_distance

    def add_start_state(self, robot_states: RobotStateCollection):
        """ Adds the starting RobotState to the RRTs tree.

        Args:
            robot_states: RobotStateCollection
        """

        robot_states.insert(self.start_state)

    def add_goal(self, robot_states: RobotStateCollection):
        """ Adds the goal node to the RRTs tree.

        Args:
            robot_states: RobotStateCollection
        """

        nearest_to_goal = robot_states.nearest(self.goal_state)
        robot_states.insert(nearest_to_goal)
        robot_states.add_edge(self.goal_state, nearest_to_goal)

    @abstractmethod
    def compute_new_state(self, random_state: RobotState, nearest_neighbor: RobotState) -> RobotState:
        """ Computes a new RobotState from the nearest_neighbor in the direction of the provided random_state.

        Args:
            random_state: RobotState
            nearest_neighbor: RobotState

        Returns:
            RobotState
        """

        ...  # pragma no cover

    @abstractmethod
    def build_tree(self) -> RobotStateCollection:
        """ Builds RRT.

        Returns:
            RobotStateCollection
        """

        ...  # pragma no cover

    @abstractmethod
    def convert_tree_to_path(self, robot_states: RobotStateCollection) -> Path:
        """ Converts the provided tree into a Path.

        Args:
            robot_states: RobotStateCollection

        Returns:
            Path
        """

        ...  # pragma no cover
