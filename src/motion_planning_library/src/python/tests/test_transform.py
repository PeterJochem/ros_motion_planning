from transform import Transform
import rospy
import geometry_msgs.msg 
import pytest
from mock import patch
import numpy as np


@pytest.fixture
def identity_transform() -> Transform:

    return Transform("a parent", "a child", 0., 0., 0., 0., 0., 0.)


def test_to_ros(identity_transform: Transform):
    
    with patch.object(rospy.Time, 'now', return_value=None):
        ros_transform = identity_transform.to_ros()
    
    assert isinstance(ros_transform, geometry_msgs.msg.TransformStamped)


def test_to_pose_vector():

    x, y, z = 1., 2., 3.
    roll, pitch, yaw = 1.1, 2.2, 3.0

    transform = Transform("a parent", "a child", x, y, z, roll, pitch, yaw)
    pose_vector = transform.to_pose_vector()

    assert isinstance(pose_vector, np.ndarray)
    assert pose_vector[0] == pytest.approx(x)
    assert pose_vector[1] == pytest.approx(y)
    assert pose_vector[2] == pytest.approx(z)
    assert pose_vector[3] == pytest.approx(roll)
    assert pose_vector[4] == pytest.approx(pitch)
    assert pose_vector[5] == pytest.approx(yaw)
