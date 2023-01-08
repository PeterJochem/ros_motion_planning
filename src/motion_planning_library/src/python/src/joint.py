from typing import List, Tuple
from transform import Transform
import sympy

class Axis:
    """ Represents the direction of rotation about a joint. 

        For example, if self.vector = [1, 0, 0] then this indicates 
        that the frame rotates about its X-axis. 
        For more details, see http://wiki.ros.org/urdf/XML/joint
     """

    def __init__(self, vector: Tuple[float, float, float]):
        """ Constrcutor. 

            Args:
                vector (Tuple): 3 tuple indicates direction of rotation
        """

        if vector is None or len(vector) != 3:
            raise Exception(f"Illegal input to axis. Expecting 3 tuple but got {vector}")

        self.vector = vector

    def is_pure_roll(self) -> bool:
        """ Check if this axis of rotation's only non-zero item is its roll
        
            Returns:
                bool - True if axis's only non-zero item is its roll. False otherwise.
        """

        return self.vector[0] == 1

    def is_pure_pitch(self) -> bool:
        """ Check if this axis of rotation's only non-zero item is its pitch
        
            Returns:
                bool - True if axis's only non-zero item is its pitch. False otherwise."""

        return self.vector[1] == 1

    def is_pure_yaw(self) -> bool:
        """ Check if this axis of rotation's only non-zero item is its yaw
        
            Returns:
                bool - True if axis's only non-zero item is its yaw. False otherwise."""

        return self.vector[2] == 1


class Joint:
    """ Represents a robot's joint and the movements it can undergo """

    def __init__(self, parent_frame: str, child_frame: str, x: float, y: float, z: float, roll: float, pitch: float,\
     yaw: float, axis: Axis, lower_limit: float, upper_limit: float):
        """ Constrcutor 
        
            Args:
                parent_frame (str): Name of parent frame with which we measure the child frame
                child_frame (str): Name of the frame which defines the link
                x (float): X position of link in parent frame in meters
                y (float): Y position of link in parent frame in meters
                z (float): Z position of link in parent frame in meters
                roll (float): Euler angle around parent's X-axis
                pitch (float): Euler angle around parent's Y-axis
                yaw (float): Euler angle around parent's Z-axis
                axis (Axis): Axis of rotation for the joint
                lower_limit (float): Lower joint limit
                upper_limit (float): Upper joint limit
        """

        self.parent_frame = parent_frame
        self.child_frame = child_frame
        
        self.x = x
        self.y = y
        self.z = z
        
        self.zero_angle_roll = roll
        self.zero_angle_pitch = pitch
        self.zero_angle_yaw = yaw

        self.lower_limit = lower_limit
        self.upper_limit = upper_limit

        self.zero_angle_transform = Transform(parent_frame, child_frame, x, y, z, roll, pitch, yaw)
        self.transform = Transform(parent_frame, child_frame, x, y, z, roll, pitch, yaw)
        self.axis = axis
        self.angle = 0.0

    def apply_rotation(self, radians: float) -> None:
        """ Rotate the joint frame around its axis of rotation by the number of radians 

            Args:
                radians (float): The number of radians from the zero angle to rotate the joint
        """

        if self.axis.is_pure_roll():
            self.transform.roll = self.zero_angle_roll + radians
        
        elif self.axis.is_pure_pitch():
            self.transform.pitch = self.zero_angle_pitch + radians

        elif self.axis.is_pure_yaw():
            self.transform.yaw = self.zero_angle_yaw + radians

        else:
            raise Exception("Cannot apply rotation since the axis of rotation is not a pure roll, pitch, or yaw.")
        
        self.angle = radians

    def symbolic_transformation(self) -> sympy.Matrix:
        """ Express the transformation from the joint's parent frame to itself 
            as a sympy matrix which has a symbol for the joint angle

            Returns:
                sympy matrix expressing the transform as a function of the joint angle
        """

        return self.transform.to_sympy(self.axis, self.zero_angle_roll, self.zero_angle_pitch, self.zero_angle_yaw)

    def get_transform(self) -> Transform:
        """ Get the transform from the joint's parent frame to itself

            Returns:
                Transform from joint's parent to itself
        """

        return self.transform