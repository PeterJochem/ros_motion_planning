import pytest
from mock import Mock
import numpy as np
from planning_algorithms.path import Path
from planning_algorithms.robot_state import JointState, RobotState
from planning_algorithms.time_scaler import NaiveTimeScaler


def test_scaling():

    num_joints = 10
    num_robot_states = 2
    states = []
    for i in range(num_robot_states):
        states.append(JointState(np.zeros(num_joints)))
    path = Path(states)

    time_scaler = NaiveTimeScaler()
    trajectory = time_scaler.scale(path)

    assert len(trajectory) == num_robot_states
    assert isinstance(trajectory[0][0], RobotState)
