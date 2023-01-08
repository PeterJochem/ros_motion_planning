import pytest
import numpy as np
from planning_algorithms.rrt import RRT
from UR5 import UR5
from diff_drive_robot import Turtlebot
from copy import copy
from planning_algorithms.planning_request import PlanningRequest
from planning_algorithms.robot_state import JointState, KDTree, PlanarPositionPose


@pytest.mark.skip(reason="This takes to long to be practical.")
def test_rrt_solver_with_ur5():

    robot = UR5()

    start_state_config = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    start_state = JointState(start_state_config)

    goal_state = copy(start_state_config)
    goal_state[0] = goal_state[0] + np.pi/(2.0 * 10)
    goal_state = JointState(goal_state)

    request = PlanningRequest(start_state, goal_state)
    request.robot_state_type = JointState
    request.get_robot_state_collection = lambda: KDTree(dimension=robot.num_joints())
    request.delta_magnitude = 0.25
    request.stopping_distance = 0.145

    myRRT = RRT(robot, request)
    myRRT.plan_path()


@pytest.mark.skip(reason="This takes to long to be practical.")
def test_rrt_solver_with_turtlebot():

    robot = Turtlebot()

    start_state_config = [0.1, 0.2]
    start_state = PlanarPositionPose(start_state_config)

    goal_state = copy(start_state_config)
    goal_state[0] = -1.
    goal_state[1] = 1.
    goal_state = PlanarPositionPose(goal_state)

    request = PlanningRequest(start_state, goal_state)
    request.robot_state_type = PlanarPositionPose
    request.get_robot_state_collection = lambda: KDTree(dimension=2)
    request.delta_magnitude = 0.25
    request.stopping_distance = 0.145

    myRRT = RRT(robot, request)
    myRRT.plan_path()
