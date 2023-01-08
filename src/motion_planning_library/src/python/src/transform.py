import rospy
import geometry_msgs.msg
from typing import List
from utilities import euler_to_quaternion
import fcl
import numpy as np
from scipy.spatial.transform import Rotation
import sympy


class Transform:
    """ Represents a position and orienation in space measured in its parent frame"""

    def __init__(self, parent_frame: str, child_frame: str, x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
        """ Constructor
        
            Args:
                parent_frame (str) - Name of frame this transform is measured in
                child_frame (str) - Name of the resulting frame
                x: (float) - X position of transform in meters measured in parent frame 
                y (float) - Y position of transform in meters measured in parent frame
                z (float) -  Z position of transform in meters in parent frame
                roll (float) -  Roll in radians measured in the parent frame 
                pitch (float) - Pitch in radians measured in the parent frame
                yaw (float) - Yaw in radians measured in the parent frame
        """

        self.parent_frame = parent_frame
        self.child_frame = child_frame
        
        self.x = x
        self.y = y
        self.z = z
        
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def to_numpy(self) -> np.ndarray:
        """ Convert to a numpy 2D array

            Returns:
                The equivalent numpy (4,4) array 
        """

        
        np_array = np.zeros((4,4))
        np_array[0:3, 0:3] = self.rotation_matrix()
        
        np_array[0][3] = self.x
        np_array[1][3] = self.y
        np_array[2][3] = self.z
        np_array[3][3] = 1.0

        return np_array 

    def rotation_matrix(self) -> np.ndarray:
        """ Get the rotation matrix for this transform
        
            Returns:
                3x3 rotation rotation matrix
        """

        rotation = Rotation.from_euler('xyz', [self.roll, self.pitch, self.yaw], degrees=False)
        return rotation.as_matrix()


    def to_ros(self) -> geometry_msgs.msg.TransformStamped:
        """ Convert this Transform to a format displayable in RVIZ 
        
            Returns:
                This transform as a geometry_msgs.msg.TransformStamped
        """

        transform = geometry_msgs.msg.TransformStamped()
        transform.header.frame_id = self.parent_frame
        transform.header.stamp = rospy.Time.now()
        transform.child_frame_id = self.child_frame
        
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = self.z

        transform.transform.rotation = euler_to_quaternion(self.roll, self.pitch, self.yaw)
        return transform

    def to_fcl(self) -> fcl.Transform:
        """ Convert to an fcl.Transform

            Returns:
                The equivalent fcl.Transform
        """

        rotation = Rotation.from_euler('xyz', [self.roll, self.pitch, self.yaw], degrees=False)
        rotation_matrix = rotation.as_matrix()
        position = np.array([self.x, self.y, self.z])
        return fcl.Transform(rotation_matrix, position)

    def to_sympy(self, axis: "Axis"=None, zero_angle_roll: float=0.0, zero_angle_pitch: float=0.0, zero_angle_yaw: float=0.0) -> sympy.Matrix:
        """ Derive the transformation as a function of the joint angle, theta 
        
            Equations are taken from the link below
            https://math.stackexchange.com/questions/3242495/yaw-pitch-and-roll-composition
        
            Args:
                axis - defines how the child frame moves relative to the parent frame
                zero_angle_roll - the roll from parent to child frame when the joint is at 0 rads 
                zero_angle_pitch - the pitch from parent to child frame when the joint is at 0 rads 
                zero_angle_yaw - the yaw from parent to child frame when the joint is at 0 rads 

            Returns:
                sympy matrix expressing the transformation as a function of the joint angle, theta
        """

        if axis is None:
            # Rigid links don't rotate relative to their parents
            return sympy.Matrix(self.to_numpy())

        symbol_name = f"theta_{self.name()}"
        theta = sympy.Symbol(symbol_name)

        if axis.is_pure_roll():
            roll = zero_angle_roll + theta
            row1 = [1.0, 0.0, 0.0, self.x]
            row2 = [0.0, sympy.cos(roll), -sympy.sin(roll), self.y]
            row3 = [0.0, sympy.sin(roll), sympy.cos(roll), self.z]
        elif axis.is_pure_pitch():
            pitch = zero_angle_pitch + theta
            row1 = [sympy.cos(pitch), 0.0, sympy.sin(pitch), self.x]
            row2 = [0.0, 1.0, 0.0, self.y]
            row3 = [-sympy.sin(pitch), 0.0, sympy.cos(pitch), self.z]
        elif axis.is_pure_yaw():
            yaw = zero_angle_yaw + theta
            row1 = [sympy.cos(yaw), -sympy.sin(yaw), 0.0, self.x]
            row2 = [sympy.sin(yaw), sympy.cos(yaw), 0.0, self.y]
            row3 = [0.0, 0.0, 1.0, self.z]
        else:
            raise Exception("Cannot compute the symbolic transformation since the axis of rotation is not a pure roll, pitch, or yaw.")

        row4 = [0.0, 0.0, 0.0, 1.0]
        return sympy.Matrix([row1, row2, row3, row4])

    def to_pose_vector(self) -> np.ndarray:
        """ Convert to a pose as a numpy array 
        
            Returns:
                numpy array = [x,y,z,roll,pitch,yaw]
        """

        return np.array([self.x, self.y, self.z, self.roll, self.pitch, self.yaw])

    def to_symbolic_pose_vector() -> sympy.Matrix:
        """ Convert to a pose as a sympy matrix
        
            Returns:
                sympy matrix = [x,y,z,roll,pitch,yaw]
        """

        raise NotImplementedError("")


    def name(self) -> str:
        """ Get name of the joint

            Returns:
                The joint's name
        """

        return f"{self.parent_frame}_{self.child_frame}"

    def __mul__(self, transform: "Transform") -> "Transform":
        """ Define multiplication between Transforms 

            Args:
                transform - The transform whose child frame we want to
                now measure in left hand side's parent frame 

            Returns:
                New transform measuring the right hand side's child frame in
                the left hand side's parent frame
        """

        result_numpy = np.matmul(self.to_numpy(), transform.to_numpy())
        resulting_rotation_matrix = Rotation.from_matrix(result_numpy[0:3, 0:3])
        angles = resulting_rotation_matrix.as_euler('xyz', degrees=False)

        result_roll = angles[0]
        result_pitch = angles[1]
        result_yaw = angles[2]

        result_x = result_numpy[0][3]
        result_y = result_numpy[1][3]
        result_z = result_numpy[2][3]

        parent_frame = self.parent_frame
        child_frame = transform.child_frame + "_modified"

        return Transform(parent_frame, child_frame, result_x, result_y, result_z, result_roll, result_pitch, result_yaw)