import numpy as np
import pytest
from mock import Mock
from planning_algorithms.robot_state import RobotStateCollection, JointState, NaiveRobotStateCollection, KDTree


@pytest.mark.parametrize("collection", [NaiveRobotStateCollection(), KDTree(dimension=6)])
def test_size(collection: RobotStateCollection):

    assert collection.size() == 0

    joint_state1 = JointState(np.zeros(6))
    joint_state2 = JointState(np.zeros(6))

    collection.insert(joint_state1)
    assert collection.size() == 1
    collection.insert(joint_state2)
    assert collection.size() == 2


def test_add_edge():

    collection = NaiveRobotStateCollection()
    collection.add_edge(Mock(), Mock())

@pytest.mark.parametrize("robot_state1, robot_state2, expected_distance", [
    (JointState(np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], dtype=np.float)),
     JointState(np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], dtype=np.float)), 0.),
    (JointState(np.array([0., 0., 0., 0., 0., 0.], dtype=np.float)),
     JointState(np.array([1., 2., 3., 4., 5., 6.], dtype=np.float)), 9.539392014169456)
                                         ])
def test_distance(robot_state1, robot_state2, expected_distance: float):

    collection = NaiveRobotStateCollection()

    collection.insert(robot_state1)
    collection.insert(robot_state2)

    assert collection.distance(robot_state1, robot_state2) == pytest.approx(expected_distance)


@pytest.mark.parametrize("collection", [NaiveRobotStateCollection(), KDTree(dimension=6)])
def test_nearest(collection: RobotStateCollection):

    joint_state1 = JointState(np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], dtype=np.float))
    joint_state2 = JointState(np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6 + 0.1], dtype=np.float))
    joint_state3 = JointState(np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6 + 100.0], dtype=np.float))

    assert collection.nearest(joint_state1) is None
    collection.insert(joint_state1)
    assert joint_state1 == collection.nearest(joint_state3)

    collection.empty()
    collection.insert(joint_state2)
    collection.insert(joint_state3)
    assert collection.nearest(joint_state1) == joint_state2
