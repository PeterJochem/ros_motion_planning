from link import Link
from joint import Joint
from transform import Transform
from typing import List
import fcl
from settings import ROOT_FRAME_NAME
from abc import ABC, abstractmethod
from copy import deepcopy
from utilities import lookup_parent_transform


class CollisionChecker(ABC):
    """ Base class for implementing collision checking """

    @abstractmethod
    def __init__(self, *args, **kwargs):
        ...  # pragma no cover

    @abstractmethod
    def check(self, *args, **kwargs) -> bool:
        ...  # pragma no cover


class RobotInternalCollisionChecker(CollisionChecker):
    """ Check for collisions between the links of a robot """
    
    def __init__(self, links: List[Link], joints: List[Joint]):
        """ Constructor

            Args:
                links - A list of links defined for the robot
-               joints - A list of transforms defined on the robot
        """

        self.links = deepcopy(links)
        self.joints = deepcopy(joints)
        self.transforms = self.get_transforms()            
    
    def get_transforms(self) -> List[Transform]:
        """" Get all the transforms defined on the links and joints

            Returns:
                List[Transform] - The list of all the transforms defined on the links and joints
        """

        link_transforms = [link.transform for link in self.links]
        joint_transforms = [joint.transform for joint in self.joints]
        return link_transforms + joint_transforms


    def set_joints(self, joint_angles: List[float]) -> None:
        """ Set the robot's joint angles 

            Args:
                joint_angles (List[float]): The robot's joint angles 
         """
        
        if len(joint_angles) != len(self.joints):
            raise RuntimeError(f"Cannot set joint angles. Robot has {len(self.joints)} \
            joints but vector has {len(joint_angles)} items.")

        for joint, joint_angle in zip(self.joints, joint_angles):
            joint.apply_rotation(joint_angle)

    def check(self, joint_angles: List[float]) -> bool:
        """ Check for collisions at the given joint angles

            Args:
                joint_angles: joint angles of robot for the collision check

            Returns:
                True if the robot is in collision. False otherwise
        """
        
        self.set_joints(joint_angles)

        num_links = len(self.links)
        for i in range(num_links):
            for j in range(i + 1, num_links):
                
                if self.check_links_for_collision(i, j):
                    return True
        
        return False

    def to_root_frame(self, transform: Transform) -> Transform:
        """ Convert the measurement of the transform into the root frame 

            Measure the child frame of the transform in the root frame of 
            all the self.transforms 

            Args:
                transform - The transform whose child frame we will meausre in the root frame

            Returns:
                Transform - Measures the child frame of transform in the root frame of self.frames
        """
    
        current_transform = transform 
        while current_transform.parent_frame != ROOT_FRAME_NAME:
            
            parent_transform = lookup_parent_transform(self.transforms, current_transform.parent_frame) 
            current_transform = parent_transform * current_transform
        
        return current_transform

    @abstractmethod
    def check_links_for_collision(self, i: int, j: int) -> bool:    
        """ Check links at indexes i and j in self.links for collision
        
            Args:
                i - index in self.links
                j - index in self.links

            Returns:
                True if the links are in collision. False otherwise.
        """

        ...  # pragma no cover

class RobotExternalObjectsCollisionChecker(CollisionChecker):
    """ Check for collisions between the links of a robot and the items around it """
    
    def __init__(self):
        """ """
        
        ...


    def check(self) -> bool:
        """ """

        ...


class FCLRobotInternalCollisionChecker(RobotInternalCollisionChecker):

    def __init__(self, links: List[Link], joints: List[Joint]):
        """ Constructor

            Args:
                links - A list of links defined for the robot
-               joints - A list of transforms defined on the robot
        """

        super().__init__(links, joints)
        self.collision_objects = [link.collision_object for link in self.links]

    def check_links_for_collision(self, i: int, j: int) -> bool:
        """ Check links at indexes i and j in self.links for collision
        
            Args:
                i - index in self.links
                j - index in self.links

            Returns:
                True if the links are in collision. False otherwise.
        """

        request = fcl.CollisionRequest(enable_contact=True)
        response = fcl.CollisionResult()

        link_i_fcl_transform = self.to_root_frame(self.links[i].transform).to_fcl()
        link_j_fcl_transform = self.to_root_frame(self.links[j].transform).to_fcl()

        fcl_object_i = self.collision_objects[i]
        fcl_object_j = self.collision_objects[j]

        fcl_object_i.setTransform(link_i_fcl_transform)
        fcl_object_j.setTransform(link_j_fcl_transform)

        fcl.collide(fcl_object_i, fcl_object_j, request, response)
        
        return response.is_collision


class MockCollisionChecker(CollisionChecker):
    """ """

    def __init__(self):
        """ """

        ...

    def check(self, joint_angles: List[float]) -> bool:
        """ """

        return False
