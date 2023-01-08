import pytest 
from planning_algorithms.trajectory import Trajectory


def test_trajectory_construction():

    path = None
    time_scaler = None
    trajectory = Trajectory(path)
