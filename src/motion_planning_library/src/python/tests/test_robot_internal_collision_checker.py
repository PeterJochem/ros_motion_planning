from CollisionChecker import FCLRobotInternalCollisionChecker
import pytest
from mock import Mock
from link import Link
from joint import Joint
from transform import Transform
from settings import ROOT_FRAME_NAME
import fcl
import numpy as np

@pytest.fixture
def fake_tiny_fcl_collision_object() -> fcl.BVHModel:
    
    mesh = fcl.BVHModel()
    
    vertices = np.array([[0, 0, 0], [1, 1, 1], [2, 2, 2]])
    triangles = np.array([[0, 1, 2]])
    mesh.beginModel(len(vertices), len(triangles))
    mesh.addSubModel(vertices, triangles)
    mesh.endModel()
    
    return mesh

@pytest.fixture
def mocked_fcl_collision_checker() -> FCLRobotInternalCollisionChecker:

    mock_collision_object = Mock()
    mock_collision_mesh = Mock(spec=Transform, fcl_collision_info=mock_collision_object)
    mock_transform = Mock(spec=Transform)

    mock_joints = [Mock(spec=Joint, transform=mock_transform)]
    mock_links = [Mock(spec=Link, transform=mock_transform, collision_mesh=mock_collision_mesh)]

    return FCLRobotInternalCollisionChecker(mock_links, mock_joints)


def test_that_check_raises_an_exception_when_given_illegal_list_of_joints(mocked_fcl_collision_checker):

        with pytest.raises(RuntimeError):
            illegal_length_joint_angles = [0.] * 10
            mocked_fcl_collision_checker.check(illegal_length_joint_angles)

        with pytest.raises(RuntimeError):
            illegal_joint_angles = None
            mocked_fcl_collision_checker.check(illegal_length_joint_angles)
        

def test_to_root_frame():

    root_to_child1 = Transform(ROOT_FRAME_NAME, "child_1", 0.,0.,0.,1.,2.,3.)

    child1_to_child_2 = Transform("child1", "child_2", 0.,0.,0.,1.,2.,3.)
    

    
