from UR5 import UR5
import numpy as np
import sympy
import math
from functools import lru_cache
import pytest


complicated_joint_angles = [1.0, 2.0, 0.0, 0.0, 0.5, -2.5]

def define_expected_transform_with_all_joints_at_zero_angles():

    expected_row1 = np.array([-1.0000000e+00, 1.2246468e-16,  4.4408921e-16,  1.8173])
    expected_row2 = np.array([1.2246468e-16, 1.0000000e+00,  1.2246468e-16,  0.21015])
    expected_row3 = np.array([-4.4408921e-16, 1.2246468e-16, -1.0000000e+00, -0.005491])
    expected_row4 = np.array([0.0, 0.0, 0.0, 1.0])
    return np.array([expected_row1, expected_row2, expected_row3, expected_row4])

def define_expected_transform_with_joints_at_comlicated_joint_angles():

    expected_row1 = np.array([0-0.1289090, -0.8462570,  0.5169444, 0.592426])
    expected_row2 = np.array([-0.9116414,  0.3062765,  0.2740524, -0.268694])
    expected_row3 = np.array([-0.3902466, -0.4359401, -0.8109648, -0.658606])
    expected_row4 = np.array([0.0, 0.0, 0.0, 1.0])
    return np.array([expected_row1, expected_row2, expected_row3, expected_row4])

@pytest.fixture
@lru_cache(1)
def ur5() -> UR5:
    """ Create a UR5 robot object 
    
        Returns:
            UR5 robot
    """

    return UR5()

def test_UR5_construction_doesnt_raise_exception():

    my_robot = UR5()

def test_get_all_joints_returns_correct_number_of_joints(ur5: UR5):

    joints = ur5.get_all_joints()
    assert len(joints) == 6

def test_get_all_links_returns_correct_number_of_links(ur5: UR5):

    joints = ur5.get_all_links()
    assert len(joints) == 7 # 6 on robot + the world

def test_get_all_links_returns_correct_number_of_links(ur5: UR5):

    joints = ur5.get_all_links()
    assert len(joints) == 7 # 6 on robot + the world

def test_set_joints_to_legal_values(ur5: UR5):

    joint_angles = [1.0] * ur5.num_joints()
    ur5.set_joints(joint_angles)
    for joint_angle, joint in zip(joint_angles, ur5.joints):
        joint_angle == pytest.approx(joint.angle)

def test_set_joints_to_illegal_values_raises_exception(ur5: UR5):

    illegal_joint_angles = [math.pi/2] * (ur5.num_joints() + 1)
    with pytest.raises(RuntimeError):
        ur5.set_joints(illegal_joint_angles)

def test_get_symbolic_transforms(ur5: UR5):

    symbolic_transforms = ur5.get_symbolic_transforms()
    assert isinstance(symbolic_transforms, list)
    assert len(symbolic_transforms) > 0
    assert isinstance(symbolic_transforms[0], sympy.Matrix)
    
    symbols = set()
    for transform in symbolic_transforms:
        symbols = symbols.union(transform.free_symbols)

    assert len(symbols) > 0

def test_collision_detection_when_ur5_is_in_self_collision(ur5: UR5):

    joint_angles_in_self_collision = [0.,0.,math.pi,0.,0.,0.] 
    ur5.set_joints(joint_angles_in_self_collision)
    assert ur5.is_current_state_in_collision()

def test_get_num_links_returns_correct_number_of_links(ur5: UR5):

    assert ur5.num_links() == 7 # 6 on robot + the world

@pytest.mark.parametrize("joint_angles, expected_numpy_array", [
    #([0.0] * 6, define_expected_transform_with_all_joints_at_zero_angles()),
    (complicated_joint_angles, define_expected_transform_with_joints_at_comlicated_joint_angles()),
    ])
def test_forward_kinematics(joint_angles, expected_numpy_array, ur5: UR5):

    transform = ur5.forward_kinematics_from_joint_angles(joint_angles)
    assert np.allclose(transform.to_numpy(), expected_numpy_array, atol=0.001)

def test_create_joint_name_to_angle_dict(ur5: UR5):

    joint_angles = [1.0] * ur5.num_joints()
    my_dict = ur5.create_joint_name_to_angle_dict(joint_angles)
    assert isinstance(my_dict, dict)
    assert len(my_dict) == len(joint_angles)
    
def test_create_joint_name_to_angle_dict_with_illegal_input_raises_exception(ur5: UR5):

    joint_angles = [1.0] * (ur5.num_joints() + 1)

    with pytest.raises(RuntimeError):
        my_dict = ur5.create_joint_name_to_angle_dict(joint_angles)

def test_get_transforms_in_order(ur5: UR5):

    transforms = ur5.get_transforms_in_order()

    for i in range(1, len(transforms)):
        parent_transform = transforms[i - 1]
        child_transform = transforms[i]
        assert parent_transform.child_frame == child_transform.parent_frame 

def test_forward_kinematics_with_illegal_input_raises_runtime_error(ur5: UR5):

    my_robot = UR5()
    with pytest.raises(RuntimeError):
        illegal_joint_angles = [0.0]
        my_robot.forward_kinematics_from_joint_angles(illegal_joint_angles)

@pytest.mark.skip(reason="This takes too long to compute.")
def test_jacobian(ur5: UR5):

    num_joints = ur5.num_joints()
    joint_angles = [0.0] * num_joints
    
    jacobian = ur5.jacobian(joint_angles)
    
    assert isinstance(jacobian, np.ndarray)
    assert(jacobian.shape == (3, num_joints))

pytest.mark.skip(reason="This takes too long to compute.")
def test_inverse_kinematics(ur5: UR5):
   """ Find a way to use mocks to do this """

   ...

