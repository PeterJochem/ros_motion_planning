from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose
from os.path import exists
import random
from typing import List
from transform import Transform
import numpy as np
from FCLCollisionInfo import FCLCollisionInfo
from copy import copy, deepcopy
import fcl


class Mesh:
    """ Represents mesh file of a part of a robot """

    def __init__(self, mesh_file: str, parent_frame: str, transform: Transform):
        """ Constructor 
        
            Args:
                mesh_file - The name of the mesh file
                parent_frame - Name of the parent frame
                transform - Transform from parent frame to mesh's frame
        """

        if not exists(mesh_file):
            raise Exception(f"Can not find mesh file for mesh defined in {parent_frame} frame.")

        self.mesh_file = mesh_file
        self.parent_frame = parent_frame
        self.transform = transform

        self.marker = Marker()
        self.marker.id = self.random_id()
        self.marker.mesh_resource = f"file:///{self.mesh_file}"
        self.marker.mesh_use_embedded_materials = True  
        self.marker.type = self.marker.MESH_RESOURCE
        self.marker.header.frame_id = self.parent_frame 
        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0
        
        mesh_pose = Pose() # The identity pose?
        
        mesh_pose.orientation.x = 0.0
        mesh_pose.orientation.y = 0.0
        mesh_pose.orientation.z = 0.0
        mesh_pose.orientation.w = 1.0

        self.marker.pose = mesh_pose


    def has_valid_pose(self) -> bool:
        """ Check if the mesh has a valid pose 
        
            Returns: True if the marker has a pose. False otherwise
        """

        return self.marker.pose is not None


    def random_id(self) -> int:
        """ Generate a random int to be an id 

            Returns:
                A random int
        """

        return int(random.getrandbits(31))

    def to_ros(self) -> Marker:   
        """ Get mesh as a ROS Marker

            Returns:
                The mesh's ROS Marker
        """
        
        return self.marker

    @staticmethod
    def create_visualization(links: List["Link"]) -> MarkerArray:
        """ Create ROS Marker array for the given set of meshes in links
        
            Args:
                links - list of links to gather meshes for

            Returns:
                A MarkerArray with a mesh for each link
        """

        array = MarkerArray()
        for link in links:
            mesh = link.get_mesh()
            if mesh is not None and mesh.has_valid_pose():
                array.markers.append(mesh.to_ros())
        return array


class VisualMesh(Mesh):
    """ Represents the visual mesh of a robot. This is what you see in RVIZ. """

    def __init__(self, mesh_file: str, parent_frame: str, transform: Transform):
        """ Constructor

            Args:
                mesh_file - The name of the mesh file
                parent_frame - Name of the parent frame
                transform - transfrom from parent frame to the mesh 
        """

        super().__init__(mesh_file, parent_frame, transform)

class CollisionMesh(Mesh):
    """ Represents the collision mesh of a robot. This is what is used for collision checking computations """

    def __init__(self, mesh_file: str, parent_frame: str, transform: Transform):
        """ Constructor

            Args:
                mesh_file - The name of the mesh file
                parent_frame - Name of the parent frame
                transform - transfrom from parent frame to the mesh 
        """

        super().__init__(mesh_file, parent_frame, transform)
        self.fcl_collision_info = FCLCollisionInfo(mesh_file)

    def __deepcopy__(self, memo: dict) -> "CollisionMesh":
        """ Implements deep copy of the object

            This is required because the fcl info object
            cannot be deep copied directly. 

            Returns:
                The deep-copied CollisionMesh
        """
        
        mesh_file = self.mesh_file
        parent_frame = self.parent_frame
        transform = self.transform

        return type(self)(mesh_file, parent_frame, transform)

    @property
    def collision_object(self) -> fcl.CollisionObject:
        """ """

        return self.fcl_collision_info.collision_object