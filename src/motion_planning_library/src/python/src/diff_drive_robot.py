import numpy as np
import sympy
from typing import List
from robot import Robot
from transform import Transform
from link import Link
from joint import Joint, Axis
from CollisionChecker import MockCollisionChecker
from settings import ROOT_FRAME_NAME, TURTLEBOT_MESH_FILE_PATH


class Turtlebot(Robot):
    """
    Implements Robot interface for a Turtlebot differential drive robot.
    """

    def __init__(self):
        """ Constructor. """

        self.mesh_file_path = TURTLEBOT_MESH_FILE_PATH
        self.links = self.get_all_links()
        self.joints = self.get_all_joints()
        self.transforms = self.get_numerical_transforms()
        self.collision_checker = MockCollisionChecker()

    def get_numerical_transforms(self) -> List[Transform]:
        """ """

        link_transforms = [link.transform for link in self.links]
        joint_transforms = [joint.transform for joint in self.joints]
        return link_transforms + joint_transforms

    def set_pose(self, pose: np.array):
        """Sets the robots pose.

        Args:
             pose: np.ndarray
                Pose vector for robot's base link.
        """

        x, y, z, roll, pitch, yaw = pose
        link_name = "base"
        visual_mesh_file, collision_mesh_file = self.get_mesh_names(link_name)
        base_link = Link(ROOT_FRAME_NAME, link_name, x, y, z, roll, pitch, yaw, visual_mesh_file, collision_mesh_file)
        self.links[0] = base_link

    def get_all_links(self) -> List[Link]:
        """Gets a list of all the robot's links.

        Returns:
            List[Link]
                The robot's links.
        """

        link_name = "base"
        visual_mesh_file, collision_mesh_file = self.get_mesh_names(link_name)
        return [Link(ROOT_FRAME_NAME, link_name, 1.0, 0.0, 0, 0, 0, 0, visual_mesh_file, collision_mesh_file)]

    def get_all_joints(self) -> List[Joint]:
        """Gets a list of all the robot's Joints.

        Returns:
            List[Joint]
        """

        return [self.define_left_wheel_joint(), self.define_right_wheel_joint()]

    def define_left_wheel_joint(self) -> Joint:
        """Defines the robot's left wheel Joint.

        Returns:
            Joint
        """

        axis = Axis([0, 1, 0])
        return Joint("base", "left_wheel_joint", 0., -0.225, 0.0, 0., 0., 0., axis, float('-inf'), float('inf'))

    def define_right_wheel_joint(self) -> Joint:
        """Defines the robot's right wheel Joint.

        Returns:
             Joint
        """

        axis = Axis([0, 1, 0])
        return Joint("base", "right_wheel_joint", 0., 0.225, 0.0, 0., 0., 0., axis, float('-inf'), float('inf'))

    def get_transforms_in_order(self) -> List[Transform]:
        """ Gets the list of symbolic transforms defined on the robot where entry (i) is the parent of entry (i + 1).

        Returns:
            List[Transform]
                An ordered list of Transforms.
        """

        ...  # pragma no cover

    def get_symbolic_transforms_in_order(self) -> List[sympy.Matrix]:
        """ Gets the list of sympy matrices defined on the robot where entry (i) is the parent of entry (i + 1).

        Returns:
            List[sympy.Matrix]
                An ordered list of sympy.Matrix.
        """

        ...  # pragma no cover
