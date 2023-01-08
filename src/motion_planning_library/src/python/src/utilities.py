""" For random utilities without a clear home """ 
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion
from typing import List, Tuple
import numpy as np
import sympy


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """ Convert from Euler angles to a quaternion

        Args:
            roll (float) - rotation about X-axis in radians
            pitch (float) - rotation about Y-axis in radians 
            yaw (float) - rotation about Z-axis in radians

        Returns:
            geometry_msgs Quaternion
    """

    quaterion_as_array = quaternion_from_euler(roll, pitch, yaw)

    q = Quaternion()
    q.x = quaterion_as_array[0]
    q.y = quaterion_as_array[1]
    q.z = quaterion_as_array[2]
    q.w = quaterion_as_array[3]

    return q


def rot2euler_symbolically(matrix: np.ndarray) -> Tuple[sympy.Matrix, sympy.Matrix, sympy.Matrix]:
    """ Convert a rotation matrix to sympy expressions for each Euler angle
    
        Adopted and modified from the online source below
        https://stackoverflow.com/questions/54616049/converting-a-rotation-matrix-to-euler-angles-and-back-special-case
    """
        
    pitch = -sympy.asin(matrix[2, 0])
    roll = sympy.atan2(matrix[2, 1] / sympy.cos(pitch), matrix[2, 2] / sympy.cos(pitch))
    yaw = sympy.atan2(matrix[1, 0] / sympy.cos(pitch), matrix[0, 0] / sympy.cos(pitch))
    return (roll, pitch, yaw)


def rot2euler_numerically(R: np.ndarray) -> Tuple[float, float, float]:
    """ Convert a rotation matrix to Euler angles
    
        Adopted and modified from the online source below
        https://stackoverflow.com/questions/54616049/converting-a-rotation-matrix-to-euler-angles-and-back-special-case
    """
    
    pitch = -np.arcsin(float(R[2,0]))
    roll = np.arctan2(float(R[2,1])/np.cos(pitch),float(R[2,2])/np.cos(pitch))
    yaw = np.arctan2(float(R[1,0])/np.cos(pitch),float(R[0,0])/np.cos(pitch))
    return (roll, pitch, yaw)
    

def identity_pose() -> Pose:
    """ Create identity from geometry_msgs.msg Pose "

        Returns :
            Identity geometry msg pose
    """

    pose = Pose()     
    
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0

    pose.orientation = euler_to_quaternion(0,0,0)
    
    return pose


def lookup_parent_transform(transforms_list: List["Transform"], parent_transform_name: str) -> "Transform":
        """ Find the transform whose child frame is the indicated parent_transform_name 

            Args:
                parent_transform_name - The name of the parent frame 

            Returns:
                Transform - The parent Transform
        """ 

        parent_transforms = [transform for transform in transforms_list if transform.child_frame == parent_transform_name]

        if parent_transforms is None or len(parent_transforms) == 0:
            raise RuntimeError(f"Could not find a transform with the name {parent_transforms}")
        elif len(parent_transforms) > 1:
            raise RuntimeError(f"Multiple transforms found with the name {parent_transforms}")
        
        return parent_transforms[0]


def sum_square_error(array1: np.ndarray, array2: np.ndarray) -> float:
    """ Compute the sum of square errors between the two numpy arrays 

        Args:
            array1 - a numpy array
            array2 - a numpy array

        Returns:
            float - the sum of square error between the two arrays
    """

    return np.sum(np.square(array1 - array2))


def random_number(min: float, max: float) -> float:
    """ Computes a random number in the provided interval [min, max]. """

    return np.random.uniform(min, max)
