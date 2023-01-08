from stl import mesh
import numpy as np
from typing import Tuple
import fcl
from os.path import exists

class FCLCollisionInfo:
    """ Stores Flexible Collision Library (FCL) info """
    
    def __init__(self, mesh_file: str):
        """ Parse mesh file into FCLCollisionInfo

            Args:
                mesh_file - The name of the mesh file as a string
        """

        if not self.is_legal_file(mesh_file):
            raise RuntimeError(f"Cannot parse {mesh_file} for FCL data")

        self.vertices, self.triangle_indexes = self.compute_vertices_and_triangles_from_file(mesh_file)
        self.mesh = self.create_fcl_mesh(self.vertices, self.triangle_indexes)
        self.collision_object = self.create_collision_object(self.mesh)
        
    def is_legal_file(self, mesh_file: str) -> bool:
        """ Check if the given file exists and has the right file extension for FCL 

            Args:
                mesh_file - The name of the file as a string

            Returns
                True if the file exists and has the right extension. False otherwise.
        """

        return exists(mesh_file) and self.is_legal_file_type(mesh_file)

    
    def is_legal_file_type(self, mesh_file: str) -> bool:
        """ Check if the given file can be parsed 

            Args:
                mesh_file - The name of the file as a string

            Returns:
                True if the file has the right extension for FCL. False otherwise.
        """
        
        if len(mesh_file) < 4:
            return False 
        
        ending_string = mesh_file[-4:]
        return ending_string == ".stl"


    def compute_vertices_and_triangles_from_file(self, mesh_file: str) -> Tuple[np.ndarray, np.ndarray]:
        """ Pre-process the mesh file to get the vertices and triangles for FCL computations

            Args:
                mesh_file - The name of the file as a string
            
            Returns:
                Tuple[np.ndarray, np.ndarray] - vertices, triangles
                
                vertices is a 2D numpy array where each entry is a 3 entry
                array of floats. Each array specifies a vertex in space.
                
                triangles is a 2D numpy array where each entry is a 3 entry
                array of indices of the vertices list indicating these three
                indices form a triangle.  
        """

        stl_mesh = mesh.Mesh.from_file(mesh_file)

        num_triangles = len(stl_mesh.points)
        num_vertices = num_triangles * 3

        vertices = np.zeros((num_vertices, 3))
        triangle_indexes = np.zeros((num_triangles, 3))

        for i, triangle_vertices in enumerate(stl_mesh.points):

            vertex1 = triangle_vertices[0:3]
            vertex2 = triangle_vertices[3:6]
            vertex3 = triangle_vertices[6:9]

            base_vertex_index = i * 3
            vertex1_index = base_vertex_index
            vertex2_index = base_vertex_index + 1
            vertex3_index = base_vertex_index + 2

            vertices[vertex1_index] = vertex1
            vertices[vertex2_index] = vertex2
            vertices[vertex3_index] = vertex3

            triangle_indexes[i] = np.array([vertex1_index, vertex2_index, vertex3_index])

        return vertices, triangle_indexes        

    def create_fcl_mesh(self, vertices: np.ndarray, triangle_indexes: np.ndarray) -> fcl.BVHModel:
        """ Create a FCL BVHModel mesh 

            Args:
                vertices is a 2D numpy array where each entry is a 3 entry
                array of floats. Each array specifies a vertex in space.
                
                triangles is a 2D numpy array where each entry is a 3 entry
                array of indices of the vertices list indicating these three
                indices form a triangle.

            Returns:
                The fcl mesh
        """

        mesh = fcl.BVHModel()
        mesh.beginModel(len(vertices), len(triangle_indexes))
        mesh.addSubModel(vertices, triangle_indexes)
        mesh.endModel()
        return mesh


    def create_collision_object(self, mesh: fcl.BVHModel) -> fcl.CollisionObject:
        """ Create FCL collision object 

            Args:
                mesh - FCL mesh 

            Returns:
                The FCL collision object with an identity transform
        """

        identity_transform = fcl.Transform()
        fcl_object = fcl.CollisionObject(mesh, identity_transform)
        return fcl_object