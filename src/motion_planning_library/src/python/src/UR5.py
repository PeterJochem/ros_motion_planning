""" UR5 class inherits from Robot and implements details specefic to the UR5. 
    The long term goal is for this class to parse a URDF file and populate the 
    data structures. Right now, I build the Python data structures in Python
    after having gotten the right information from the URDF itself. 
"""
import math
from typing import List, Tuple
from transform import Transform
from robot import Robot
from link import Link
from joint import Joint, Axis
from settings import ROOT_FRAME_NAME, UR5_MESH_FILE_PATH
import sympy


class UR5(Robot):
    """ UR5 class inherits from Robot and implements details specefic to the UR5.  """
    
    def __init__(self):
        """ Constructor. """

        self.mesh_file_path = UR5_MESH_FILE_PATH
        super().__init__()

    def define_base_link(self) -> Link:
        """ Defines the UR5's base link.

        Returns:
            Link
                The UR5's base link.
        """
        
        link_name = "base"
        visual_mesh_file, collision_mesh_file = self.get_mesh_names(link_name)
        return Link(ROOT_FRAME_NAME, "base_link", 1.0, 0.0, 0, 0, 0, 0, visual_mesh_file, collision_mesh_file)

    def define_shoulder_link(self) -> Link:
        """ Defines the UR5's shoulder link.

        Returns:
            Link
                The UR5's shoulder link.
        """
        
        link_name = "shoulder"
        visual_mesh_file, collision_mesh_file = self.get_mesh_names(link_name)
        return Link("base_shoulder_joint", "shoulder_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, visual_mesh_file, collision_mesh_file)

    
    def define_upper_arm_link(self) -> Link: 
        """ Defines the UR5's upper arm link.

        Returns:
            Link
                The UR5's upper arm link.
        """

        link_name = "upperarm"
        visual_mesh_file, collision_mesh_file = self.get_mesh_names(link_name)
        return Link("shoulder_upper_arm_joint", "upper_arm_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, visual_mesh_file, collision_mesh_file)

    def define_forearm_link(self) -> Link:
        """ Defines the UR5's forearm link.

        Returns:
            Link
                The UR5's forearm link.
        """

        link_name = "forearm"
        visual_mesh_file, collision_mesh_file = self.get_mesh_names(link_name)
        return Link("upper_arm_forearm_joint", "forearm_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, visual_mesh_file, collision_mesh_file)

    def define_wrist1_link(self) -> Link:
        """ Defines the UR5's first wrist link.

        Returns:
            Link
                The UR5's first wrist link.
        """

        link_name = "wrist1"
        visual_mesh_file, collision_mesh_file = self.get_mesh_names(link_name)
        return Link("forearm_wrist1_joint", "wrist1_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, visual_mesh_file, collision_mesh_file)

    def define_wrist2_link(self) -> Link:
        """ Defines the UR5's second wrist link.

        Returns:
            Link
                The UR5's second wrist link.
        """

        link_name = "wrist2"
        visual_mesh_file, collision_mesh_file = self.get_mesh_names(link_name)
        return Link("wrist1_wrist2_joint", "wrist2_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, visual_mesh_file, collision_mesh_file)

    def define_wrist3_link(self) -> Link:
        """ Defines the UR5's third wrist link.

        Returns:
            Link
                The UR5's third wrist link.
        """

        link_name = "wrist3"
        visual_mesh_file, collision_mesh_file = self.get_mesh_names(link_name)
        return Link("wrist2_wrist3_joint", "wrist3_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, visual_mesh_file, collision_mesh_file)

    def define_end_effector_transform(self) -> Transform:
        """ Describe me """
        
        parent_frame = "wrist3_link"
        child_frame = "end_effector_frame"
        return Transform(parent_frame, child_frame, 0.0, 0.10, 0.0, 0.0, 0.0, 0.0)

    def define_base_shoulder_joint(self) -> Joint:
        """ Define the UR5's base to shoulder joint.

        Returns:
            Joint
                The base to shoulder joint.
        """

        axis = Axis([0,0,1])
        return Joint("base_link", "base_shoulder_joint", 0., 0., 0.089159, 0., 0., 0., axis, -2 * math.pi, 2 * math.pi)

    def define_shoulder_upper_arm_joint(self) -> Joint:
        """ Defines the UR5's shoulder to upper arm joint.

        Returns:
            Joint
                The shoulder to upper arm joint.
        """

        axis = Axis([0,1,0])
        return Joint("shoulder_link", "shoulder_upper_arm_joint", 0., 0.13585, 0., 0., math.pi/2, 0., axis, -2 * math.pi, 2 * math.pi)

    def define_upper_arm_forearm_joint(self) -> Joint:
        """ Defines the UR5's upper arm to forearm joint.

        Returns:
            Joint
                The upper arm to forearm joint.
        """

        axis = Axis([0,1,0])
        delta = -0.001  
        return Joint("upper_arm_link", "upper_arm_forearm_joint", 0., -0.1197 + delta, 0.42500, 0., 0., 0., axis, -2 * math.pi, 2 * math.pi)

    def define_forearm_wrist1_joint(self) -> Joint:
        """ Defines the UR5's forearm to wrist1 joint.

        Returns:
            Joint
                The forearm to wrist joint.
        """

        axis = Axis([0,1,0])
        delta = 0.001
        return Joint("forearm_link", "forearm_wrist1_joint", 0., delta, 0.39225, 0., math.pi/2, 0., axis, -2 * math.pi, 2 * math.pi)

    def define_wrist1_wrist2_joint(self) -> Joint:
        """ Defines the UR5's wrist1 to wrist2 joint.

        Returns:
            Joint
                The wrist1 to wrist2 joint.
        """

        axis = Axis([0,0,1])
        return Joint("wrist1_link", "wrist1_wrist2_joint", 0., 0.093, 0., 0., 0., 0., axis, -2 * math.pi, 2 * math.pi)

    def define_wrist2_wrist3_joint(self):
        """ Defines the UR5's wrist2 to wrist3 joint.

        Returns:
            Joint
                The wrist2 to wrist3 joint.
        """

        axis = Axis([0,1,0])
        delta = 0.001
        return Joint("wrist2_link", "wrist2_wrist3_joint", 0., delta, 0.09465, 0., 0., 0., axis, -2 * math.pi, 2 * math.pi)

    
    def get_all_joints(self) -> List[Joint]:
        """ Gets a list of all the UR5's joints.
        
        Returns:
            List[Joint]
                The UR5's joints
        """

        base_shoulder_joint = self.define_base_shoulder_joint()
        shoulder_upper_arm_joint = self.define_shoulder_upper_arm_joint()
        upper_arm_forearm_link = self.define_upper_arm_forearm_joint()
        forearm_wrist1_joint = self.define_forearm_wrist1_joint()
        wrist1_wrist2_joint = self.define_wrist1_wrist2_joint()
        wrist2_wrist3_joint = self.define_wrist2_wrist3_joint()

        return [base_shoulder_joint, shoulder_upper_arm_joint, upper_arm_forearm_link, forearm_wrist1_joint,
                wrist1_wrist2_joint, wrist2_wrist3_joint]
        
    def get_all_links(self) -> List[Link]:
        """ Gets a list of all the UR5's Links.
        
        Returns:
            List[Link]
                The UR5's links.
        """

        base_link = self.define_base_link()
        shoulder_link = self.define_shoulder_link()
        upper_arm_link = self.define_upper_arm_link()
        forearm_link = self.define_forearm_link()
        wrist1_link = self.define_wrist1_link()
        wrist2_link = self.define_wrist2_link()
        wrist3_link = self.define_wrist3_link()

        return [base_link, shoulder_link, upper_arm_link, forearm_link, wrist1_link, wrist2_link, wrist3_link]

    def get_static_world_to_base_transform(self) -> Transform:
        """ Gets the transform between the world and the robot's base.

        Returns:
            Transform
                The Transform between the robot's base and the world.
        """

        return self.define_base_link().transform

    def get_end_effector_transform(self) -> Transform:
        """ Gets the transform between the robot's last link and the end effector.

        Returns:
            Transform
                The Transform between the robot's last link and the end effector
        """

        return self.define_end_effector_transform()

    def get_end_effector_symbolic_transform(self) -> sympy.Matrix:
        """ Gets the transform between the robot's last link and the end effector expressed as a sympy matrix.

        Returns:
            Transform
                The Transform between the robot's last link and the end effector expressed as a sympy matrix
        """

        numpy_array = self.get_end_effector_transform().to_numpy()
        return sympy.Matrix(numpy_array)

    def get_transforms_in_order(self) -> List[Transform]:
        """ Gets the list of transforms defined on the robot where entry (i) is the parent of entry (i + 1).

        Returns:
            List[Transform]
                An ordered list of Transforms defined on the robot.
        """

        transforms = []
        for link, joint in zip(self.links[0:6], self.joints):
            transforms.append(link.get_transform())
            transforms.append(joint.get_transform())

        transforms.append(self.links[6].get_transform())
        transforms.append(self.get_end_effector_transform())
        return transforms
    
    def get_symbolic_transforms_in_order(self) -> List[sympy.Matrix]:
        """ Gets the list of transforms (expressed as sympy matrices) defined on the robot where index (i) is the parent
         transformation of index (i + 1).

        Returns:
            List[sympy.Matrix]
                A list of ordered transformations expressed as sympy matrices.
        """

        transforms = []
        for link, joint in zip(self.links[0:6], self.joints):
            transforms.append(link.symbolic_transformation())
            transforms.append(joint.symbolic_transformation())

        transforms.append(self.links[6].symbolic_transform())
        transforms.append(self.get_end_effector_symbolic_transform())
        return transforms
