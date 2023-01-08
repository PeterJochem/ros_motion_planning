from abc import ABC, abstractmethod
from typing import List
from .planner import Planner
from transform import Transform
from robot import Robot
from .planning_request import PlanningRequest
from .robot_state import RobotState, JointState
from .path import Path
from .trajectory import Trajectory


class OfflinePlanner(Planner, ABC):
    """ Plans robot trajectories without feedback. """

    def __init__(self, robot: Robot, request: PlanningRequest):
        """ Constructor.

        Args:
            robot: Robot
            request: PlanningRequest
        """

        self.robot = robot
        self.request = request
        if not self.is_request_legal(request):
            raise RuntimeError("Cannot instantiate this offline planner because the request is not legal.")

    def plan_to_joint_state(self, starting_state: JointState, goal_state: JointState) -> Trajectory:
        """ Plans from the provided starting joint state to the provided goal joint state.

        Args:
            starting_state: JointState
            goal_state: JointState

        Returns:
            Trajectory
        """

        path = self.plan_path(starting_state, goal_state)
        return self.scale_path(path)

    def plan_to_pose(self, starting_joint_state: JointState, goal_base_to_end_effector: Transform) -> Trajectory:
        """ Plans from the provided starting joint state to the provided goal pose.

        Args:
            starting_joint_state: JointState
            goal_base_to_end_effector: JointState

        Returns:
            Trajectory
        """

        goal_joint_state = JointState(self.robot.inverse_kinematics(goal_base_to_end_effector))
        return self.plan_to_joint_state(starting_joint_state, goal_joint_state)

    @property
    def goal_state(self) -> RobotState:
        """ Gets this instances goal RobotState."""

        return self.request.goal_state

    @property
    def start_state(self) -> RobotState:
        """ Gets this instance's starting RobotState. """

        return self.request.start_state

    @abstractmethod
    def plan_path(self) -> Path:
        """ Plans a path that satisfies this instance's Request.

        Returns:
            Path
        """

        ...  # pragma no cover
