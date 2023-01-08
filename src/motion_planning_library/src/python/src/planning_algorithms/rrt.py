import numpy as np
from typing import List
from robot import Robot
from .planning_request import PlanningRequest
from .path import Path
from .robot_state import RobotState, RobotStateCollection
from .trajectory import Trajectory
from .rrt_base import RRTBase


class RRT(RRTBase):
    """ Base class for Rapidly Exploring Random Tree variants. """

    def __init__(self, robot: Robot, request: PlanningRequest):
        """ Constructor.

        Args:
            robot: Robot
            request: PlanningRequest
        """

        super().__init__(robot, request)

    def compute_new_state(self, random_state: RobotState, nearest_neighbor: RobotState) -> RobotState:
        """ Computes a new RobotState from the nearest_neighbor in the direction of the provided random_state.

        Args:
            random_state: RobotState
            nearest_neighbor: RobotState

        Returns:
            RobotState
        """

        if random_state is None:
            raise RuntimeError("Illegal arguments to compute_new_state. State cannot be None.")

        if nearest_neighbor is None:
            return random_state
        else:
            difference = nearest_neighbor.configuration - random_state.configuration
            normalized_direction = difference / np.linalg.norm(difference)
            delta = normalized_direction * self.delta_magnitude
            new_configuration_array = nearest_neighbor.configuration + delta
            return self.robot_state_type(new_configuration_array)

    def plan_path(self) -> Path:
        """ Plans a path that satisfies this instance's Request.

        Returns:
            Path
        """

        if not self.is_request_legal(self.request):
            raise RuntimeError("Planning request is not legal.")

        robot_states = self.build_tree()
        path = self.convert_tree_to_path(robot_states)
        path.visualize()  # For testing
        return path

    def scale(self, path: Path) -> Trajectory:
        """ Scales the provided path through time.

        Args:
            path: Path

        Returns:
            Trajectory
        """

        ...

    def build_tree(self) -> RobotStateCollection:
        """ Builds a rapidly exploring random tree.

        Returns:
            RobotStateCollection
        """

        robot_states = self.request.get_robot_state_collection()
        self.add_start_state(robot_states)

        while not self.is_done(robot_states):

            random_state = self.robot_state_type.random_legal(self.robot)
            nearest_neighbor = robot_states.nearest(random_state)
            new_state = self.compute_new_state(random_state, nearest_neighbor)

            if new_state.is_legal(self.robot):
                robot_states.insert(new_state)
                robot_states.add_edge(new_state, nearest_neighbor)

        self.add_goal(robot_states)
        return robot_states

    def convert_tree_to_path(self, robot_states: RobotStateCollection) -> Path:
        """ Converts the provided tree to a Path.

        Args:
            robot_states: RobotStateCollection

        Returns:
            Path
        """

        visited_states = {self.start_state: True}
        states = self.depth_first_search(self.start_state, robot_states, visited_states)
        if states is None:
            raise RuntimeError("Failed to find a solution after building the tree.")

        states.append(self.start_state)
        states = list(reversed(states))
        return Path(states)

    def depth_first_search(self, state: RobotState, robot_states: RobotStateCollection, searched_from_states: dict) ->\
            List[RobotState]:
        """ Finds path through RRT's tree via a depth first search.

        Args:
            state: RobotState
            robot_states: RobotStateCollection
            searched_from_states: dict

        Returns:
            List[RobotState]
        """

        for next_state in robot_states.edges[state]:

            if next_state in searched_from_states:
                continue

            if next_state is self.goal_state:
                return [self.goal_state]

            searched_from_states[next_state] = True
            result = self.depth_first_search(next_state, robot_states, searched_from_states)
            if result is not None:
                result.append(next_state)
                return result

        return None
