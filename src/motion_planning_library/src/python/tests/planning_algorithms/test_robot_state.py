import numpy as np
import pytest
from planning_algorithms.robot_state import JointState, PoseState


@pytest.mark.parametrize("robot_state_subclass, configuration", [
                                                (JointState, np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], dtype=np.float)),
                                                (PoseState, np.array([0.1, 0.4, 0.6, 0.8, 1.0, 1.2], dtype=np.float))
                                                ])
def test_robot_state(robot_state_subclass, configuration: np.ndarray):
    """ """

    robot_state = robot_state_subclass(configuration)

    assert isinstance(robot_state.configuration, np.ndarray)
    assert isinstance(robot_state.configuration_as_tuple, tuple)

    for num1, num2 in zip(configuration, robot_state.configuration):
        assert num1 == pytest.approx(num2)

    for num1, num2 in zip(configuration, robot_state.configuration_as_tuple):
        assert num1 == pytest.approx(num2)


