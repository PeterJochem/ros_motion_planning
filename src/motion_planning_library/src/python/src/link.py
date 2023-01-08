from transform import Transform
from mesh import Mesh, VisualMesh, CollisionMesh
import fcl


class Link:
    """ Represents a robot's link and its geometry """

    def __init__(self, parent_frame: str, child_frame: str, x: float, y: float, z: float, roll: float, pitch: float, \
                 yaw: float, visual_mesh_file=None, collision_mesh_file=None):
        """ Constructor

            Args:
                parent_frame (str): Name of parent frame with which we measure the child frame
                child_frame (str): Name of the frame which defines the link
                x (float): X position of link in parent frame in meters
                y (float): Y position of link in parent frame in meters
                z (float): Z position of link in parent frame in meters
                roll (float): Euler angle around parent's X-axis
                pitch (float): Euler angle around parent's Y-axis
                yaw (float): Euler angle around parent's Z-axis
        """

        self.transform = Transform(parent_frame, child_frame, x, y, z, roll, pitch, yaw)
        self.collision_mesh = CollisionMesh(collision_mesh_file, child_frame, self.transform)
        self.visual_mesh = VisualMesh(visual_mesh_file, child_frame, self.transform)
        

    def get_mesh(self) -> Mesh:
        """ Get mesh file for this link

            Returns:
                Mesh - the mesh file for this link
        """

        return self.visual_mesh

    def symbolic_transformation(self):
        """ Express the transformation from the link's parent frame to itself 
            as a sympy matrix

            Returns:
                sympy matrix expressing the transform
        """

        return self.transform.to_sympy()

    def get_transform(self) -> Transform:
        """ Get the transform from the link's parent frame to itself

            Returns:
                Transform from link's parent to itself
        """

        return self.transform

    @property
    def collision_object(self) -> fcl.CollisionObject:

        return self.collision_mesh.collision_object