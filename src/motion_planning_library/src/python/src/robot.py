from abc import abstractmethod
from copy import deepcopy
import numpy as np
import sympy
from typing import List, Tuple
from functools import reduce
from utilities import rot2euler_symbolically, sum_square_error, random_number
from transform import Transform
from CollisionChecker import FCLRobotInternalCollisionChecker
from joint import Joint
from planning_algorithms.robot_state import JointState


class Robot:
    """ Base class for specific robots to inherit from. """

    def __init__(self):
        """ Constructor. """

        self.links = self.get_all_links()
        self.joints = self.get_all_joints()    
        self.transforms = self.get_numerical_transforms()
        self.collision_checker = FCLRobotInternalCollisionChecker(self.links, self.joints)

        # Compute just in time and then cache them
        self.symbolic_forward_kinematics_sympy_matrix = None
        self.symbolic_jacobian_sympy_matrix = None

    def num_links(self) -> int:
        """ Gets the number of links on the robot.

        Returns:
            The number of links on the robot.
        """

        return len(self.links)

    def num_joints(self) -> int:
        """ Gets the number of joints on the robot.

        Returns:
            The number of joints on the robot.
        """

        return len(self.joints)

    def set_joints(self, joint_angles: List[float]) -> None:
        """ Sets the robot's joint angles.

        Args:
            joint_angles: List[float]
                The robot's joint angles.
        """
        
        if len(joint_angles) != len(self.joints):
            raise RuntimeError(f"Cannot set joint angles. Robot has {len(self.joints)} \
            joints but vector has {len(joint_angles)} items.")

        for joint, joint_angle in zip(self.joints, joint_angles):
            joint.apply_rotation(joint_angle)

    def get_numerical_transforms(self) -> List[Transform]:
        """" Gets all the transforms defined on the robot.

        Returns:
            List[Transform]
                All the transforms defined on the robot.
        """

        link_transforms = [link.transform for link in self.links]
        joint_transforms = [joint.transform for joint in self.joints]
        end_effector_transform = [self.get_end_effector_transform()]
        return link_transforms + joint_transforms + end_effector_transform

    def get_symbolic_transforms(self) -> List[sympy.Matrix]:
        """" Gets a list of transformations expressed as a function of the joint angles as sympy matrices.

        Returns:
            List[sympy.Matrix]
        """

        link_transforms = [link.symbolic_transformation() for link in self.links]
        joint_transforms = [joint.symbolic_transformation() for joint in self.joints]
        end_effector_transform = [self.get_end_effector_symbolic_transform()]
        return link_transforms + joint_transforms + end_effector_transform

    def is_current_state_in_collision(self) -> bool:
        """ Checks if the robot's current joints are in collision.

        Returns:
            bool
                True if any of the robot's links are in collision. False otherwise.
        """

        joint_angles = [joint.angle for joint in self.joints]
        return self.check_for_collision(joint_angles)
    
    def check_for_collision(self, joint_angles: List[float]) -> bool:
        """ Checks if any robot link is in collision with another link.

        Args:
            joint_angles: the joint angles to check for collision

        Returns:
            bool
                True if one or more links are in collision. False otherwise.
        """

        return self.collision_checker.check(joint_angles)

    def forward_kinematics_from_joint_states(self, joints: List[Joint]) -> Transform:
        """ Computes the base frame to end-effector frame transform.

        Args:
            joints: List[Joint]

        Returns:
            Transform
                The transform from the robot's base link to its end-effector
        """

        transforms = []
        for link, joint in zip(self.links, joints):
            
            transforms.append(link.transform)
            transforms.append(joint.transform)

        transforms += [self.get_end_effector_transform()]

        return reduce(lambda transform1, transform2: transform1*transform2, transforms)
    
    def forward_kinematics_from_joint_angles(self, joint_angles: List[float]) -> Transform:
        """ Computes the base frame to end-effector frame transform.

        Args:
            joints: List[Joint]

        Returns:
            The transform from the robot's base link to its end-effector.
        """

        if joint_angles is None or len(joint_angles) != self.num_joints():
            raise RuntimeError("Cannot compute forward kinematics since input is illegal.") 

        joints = [deepcopy(joint) for joint in self.joints]
        for joint, joint_angle in zip(joints, joint_angles):
            joint.apply_rotation(joint_angle)
        return self.forward_kinematics_from_joint_states(joints)

    @property
    def symbolic_forward_kinematics(self) -> sympy.Matrix:
        """ Gets the symbolic forward kinematics as a sympy matrix.

        Returns:
            The symbolic forward kinematics as a function of the joint angles expressed as a sympy matrix
        """

        if self.symbolic_forward_kinematics_sympy_matrix is None:
            symbolic_fk = self.compute_symbolic_forward_kinematics()
            self.symbolic_forward_kinematics_sympy_matrix = symbolic_fk
        return self.symbolic_forward_kinematics_sympy_matrix

    def compute_symbolic_forward_kinematics(self) -> sympy.Matrix:
        """ Computes sympy matrix expressing the forward kinematics as of function of the joint angles.

        Returns:
            sympy matrix
                Expresses the robot's forward kinematics.
        """

        symbolic_transforms = self.get_symbolic_transforms_in_order()
        self.symbolic_forward_kinematics_sympy_matrix = sympy.prod(symbolic_transforms)
        return self.symbolic_forward_kinematics_sympy_matrix

    @property
    def symbolic_jacobian(self) -> sympy.Matrix:
        """ Gets the symbolic jacobian as a sympy matrix.

        Returns:
            sympy.Matrix
                Expresses the jacobian.
        """

        if self.symbolic_jacobian_sympy_matrix is None:
            symbolic_jacobian = self.compute_symbolic_jacobian()
            self.symbolic_jacobian_sympy_matrix = symbolic_jacobian
        return self.symbolic_jacobian_sympy_matrix

    def compute_symbolic_jacobian(self) -> sympy.Matrix:
        """ Computes the jacobian as a sympy matrix using symbols for each joint angle.

            Returns:
                sympy.Matrix
                    Expresses the jacobian.
        """

        symbolic_forward_kinematics = self.compute_symbolic_forward_kinematics()
        
        x, y, z, _ = symbolic_forward_kinematics[:, 3]
        roll, pitch, yaw = rot2euler_symbolically(symbolic_forward_kinematics) 
        
        symbolic_pose = sympy.Matrix([x, y, z, roll, pitch, yaw])
        
        joint_angle_names = self.get_joint_angle_names()
        joint_angle_symbol_vector = sympy.Matrix(joint_angle_names)
        
        return symbolic_pose.jacobian(joint_angle_symbol_vector)

    def jacobian(self, joint_angles: List[float]) -> np.ndarray:
        """ Computes the jacobian at the given joint angles.

        Args:
            joint_angles - the joint angles where we compute the jacobian

        Returns:
            The jacobian as a numpy array
        """
        
        symbolic_jacobian = self.compute_symbolic_jacobian()
        joint_name_to_angles = self.create_joint_name_to_angle_dict(joint_angles)
        jacobian_at_joint_angles = symbolic_jacobian.evalf(subs=joint_name_to_angles)
        return np.array(jacobian_at_joint_angles)

    def inverse_jacobian(self, joint_angles: List[float]) -> np.ndarray:
        """ Computes the Moore Penroose pseudo inverse jacobian at a given set of joint angles.

        Args:
            joint_angles - the joint angles where we compute the jacobian

        Returns:
            The inverse of the jacobian at a given set of joint angles
        """

        return np.linalg.pinv(self.jacobian(joint_angles))

    def create_joint_name_to_angle_dict(self, joint_angles: List[float]) -> dict:
        """ Creates a dict mapping joint names to angles.
        
        Args:
            joint_angles: List[float]

        Returns:
            dict
                Maps joint names to joint angles.
        """

        if self.num_joints() != len(joint_angles):
            raise RuntimeError("Illegal input. The number of joints and the list of angles have unequal length")

        joint_name_to_angles = {}
        joint_angle_names = self.get_joint_angle_names_in_order()
        for name, angle in zip(joint_angle_names, joint_angles):
            joint_name_to_angles[name] = angle

        return joint_name_to_angles
    
    def get_joint_angle_names_in_order(self) -> List[str]:
        """ Creates an ordered list of joint angle names.

        Returns:
            List[str]
                List of names of the robot's joints.
        """

        return [f"theta_{joint.transform.name()}" for joint in self.joints]

    def inverse_kinematics(self, transform_base_end_effector: Transform, tolerance: float=0.01, max_iterations: int=100) -> List[float]:
        """ Computes the joint angles which place the end effector at the given pose.

        This implements the Newton-Raphson numerical root finding algorithm
        The details are available in Modern Robotics, Lynch and Park et. al. See link below
        http://hades.mech.northwestern.edu/images/7/7f/MR.pdf#page=245

        Args:
            transform_base_end_effector: Transform
                The desired base to ee transform

        Returns:
            List[float]:
                List of joint angles which will make the robot's end effector
                be at the desired pose
        """

        desired_pose = transform_base_end_effector.to_pose_vector()
        theta = np.zeros(6) # How to choose this?

        iteration = 0
        current_pose = self.forward_kinematics_from_joint_angles(theta).to_pose_vector()
        error = sum_square_error(desired_pose, current_pose)

        while error > tolerance and iteration < max_iterations:

            delta_theta = np.matmul(self.inverse_jacobian(theta), (desired_pose - current_pose))
            theta += delta_theta
            current_pose = self.forward_kinematics_from_joint_angles(theta).to_pose_vector()
            error = sum_square_error(desired_pose, current_pose)
            iteration += 1

        if error > tolerance and iteration >= max_iterations:
            raise RuntimeError("Numerical method failed to converge within allowed num of iterations.")

        return theta

    def random_joint_angles(self) -> List[float]:
        """ Computes a random set of joint angles which satisfy the robot's joint limits.

        Returns:
            List[float]
        """

        random_joint_angles = []
        for joint in self.joints:
            random_joint_angle = random_number(joint.lower_limit, joint.upper_limit)
            random_joint_angles.append(random_joint_angle)
        return random_joint_angles

    def satisfies_joint_limits(self, joint_angles: List[float]) -> bool:
        """ Checks if the provided joint_angles satisfy the robot's joint limits.

        Args:
            joint_angles: List[float]

        Returns:
            bool
        """

        for joint_angle, joint in zip(joint_angles, self.joints):
            if joint_angle < joint.lower_limit or joint_angle > joint.upper_limit:
                return False
        return True

    def random_legal_joint_state(self) -> JointState:
        """ Computes a random, legal JointState.

        Returns:
            JointState
        """

        max_iterations = 100
        iteration = 0

        random_joint_state = JointState(self.random_joint_angles())
        while not random_joint_state.is_legal(self) and iteration <= max_iterations:
            random_joint_state = JointState(self.random_joint_angles())
            iteration += 1

        if iteration >= max_iterations:
            raise RuntimeError("Failed to generate a random state within the allowed num iterations.")

        return random_joint_state

    def get_mesh_names(self, link_name: str) -> Tuple[str, str]:
        """ Convert the link name to its mesh file names.

        Args:
            link_name: the name of the link

        Returns:
            Name of visual_mesh_file, Name of collision_mesh_file

        """

        visual_file_ending = ".dae"
        collision_file_ending = ".stl"
        visual_mesh_file = f"{self.mesh_file_path}/{link_name}{visual_file_ending}"
        collision_mesh_file = f"{self.mesh_file_path}/{link_name}{collision_file_ending}"
        return visual_mesh_file, collision_mesh_file

    @abstractmethod
    def get_transforms_in_order(self) -> List[Transform]:
        """ Gets the list of symbolic transforms defined on the robot where entry (i) is the parent of entry (i + 1).

        Returns:
            List[Transform]
                An ordered list of Transforms.
        """

        ...  # pragma no cover
    
    @abstractmethod
    def get_symbolic_transforms_in_order(self) -> List[sympy.Matrix]:
        """ Gets the list of sympy matrices defined on the robot where entry (i) is the parent of entry (i + 1).

        Returns:
            List[sympy.Matrix]
                An ordered list of sympy.Matrix.
        """

        ...  # pragma no cover
