import numpy as np
from abc import ABC, abstractmethod
from typing import List, Tuple
import kdtree
from utilities import sum_square_error
from utilities import random_number


class RobotState(ABC):
    """ Base class for RobotState representations. """
    
    def __init__(self):
        """ Constructor. """

        ...  # pragma no cover

    @property
    @abstractmethod
    def configuration(self) -> np.ndarray:
        """ Gets np.ndarray in configuration in space.

        Returns:
            np.ndarray
        """

        ...  # pragma no cover

    @property
    @abstractmethod
    def configuration_as_tuple(self) -> Tuple[float]:
        """ Vector in configuration in space expressed as a tuple.

            Note: This is very useful since a tuple can be hashed.

        Returns:
            Tuple[float]
        """

        ...  # pragma no cover

    def __hash__(self) -> int:
        """ Implements hashing.

        Returns:
            int
                The hash value of this instance.
        """

        return hash(self.configuration_as_tuple)

    def __eq__(self, robot_state: "RobotState") -> bool:
        """ Implements == operator.

        Args:
            robot_state: RobotState

        Returns:
            bool
        """

        return self.configuration_as_tuple == robot_state.configuration_as_tuple

    def __str__(self) -> str:
        """ Creates a string representation of this instance.

        Returns:
            str
        """

        return f"{self.__class__} with configuration {self.configuration}"


class JointState(RobotState):
    """ Represents a robot's configuration in its joint space. """

    def __init__(self, joint_angles: List[float]):
        """ Constructor.

        Args:
            joint_angles: List[float]
        """

        self._joint_angles = joint_angles

    def is_legal(self, robot: "Robot") -> bool:
        """ Checks if this JointState is legal for the provided robot.

        Args:
            robot: Robot

        Returns:
            bool
        """

        return robot.satisfies_joint_limits(self._joint_angles) and not robot.check_for_collision(self._joint_angles)

    @property
    def configuration(self) -> np.ndarray:
        """ Vector in configuration in space expressed as a np.ndarray.

        Returns:
            Tuple[float]
        """

        return self._joint_angles

    @property
    def configuration_as_tuple(self) -> Tuple[float]:
        """ Vector in configuration in space expressed as a tuple.

        Note: This is very useful since a tuple can be hashed.

        Returns:
            Tuple[float]
        """

        return tuple(self._joint_angles)

    @staticmethod
    def random_legal(robot: "Robot") -> "JointState":
        """ Creates a new, random, legal JointState for the provided robot.

        Args:
            robot: Robot

        Returns:
            JointState
        """

        return robot.random_legal_joint_state()


class PoseState(RobotState):
    """Represents a robot's state as a pose vector. """

    def __init__(self, pose_vector: np.ndarray):
        """ Constructor.

         Args:
             pose_vector: np.ndarray
         """

        self._pose_vector = pose_vector

    @property
    def configuration(self) -> np.ndarray:
        """ Vector in configuration in space expressed as a np.ndarray.

        Returns:
            np.ndarray
        """
        return self._pose_vector

    @property
    def configuration_as_tuple(self) -> Tuple[float]:
        """ Vector in configuration in space expressed as a Tuple[float].

        Returns:
            Tuple[float]
        """

        return tuple(self._pose_vector)

    def is_legal(self, robot: "Robot") -> bool:

        return True


class Pose2DState(PoseState):
    """Represents a robot's state as a pose vector. """

    def __init__(self, pose_vector: np.ndarray):
        """ Constructor.

         Args:
             pose_vector: np.ndarray
         """

        super().__init__()


class Pose3DState(PoseState):
    """Represents a robot's state as a pose vector. """

    def __init__(self, pose_vector: np.ndarray):
        """ Constructor.

         Args:
             pose_vector: np.ndarray
         """

        super().__init__()


class PlanarPositionPose(PoseState):
    """ """

    def __init__(self, position_pose):
        """ """

        super().__init__(position_pose)

    @staticmethod
    def random_legal(robot: "Robot"):
        """ """

        min_x, max_x = -2.0, 2.0
        min_y, max_y = -2.0, 2.0

        x = random_number(min_x, max_x)
        y = random_number(min_y, max_y)

        return PlanarPositionPose(np.array([x, y]))


class RobotStateCollection(ABC):
    """ Base class which stores a set of RobotStates. """

    def __init__(self):
        """ Constructor. """

        ...  # pragma no cover

    @abstractmethod
    def size(self) -> int:
        """ Gets the number of RobotStates in this collection.

        Returns:
            int
        """

        ...  # pragma no cover

    @abstractmethod
    def insert(self, state: RobotState):
        """ Inserts a new RobotState into this collection.

        Args:
             state: RobotState
        """

        ...  # pragma no cover

    @abstractmethod
    def empty(self):
        """ Empties or removes all RobotStates from the collection."""

        ...  # pragma no cover

    @abstractmethod
    def nearest(self, state: RobotState) -> RobotState:
        """ Finds the nearest RobotState to the provided state.

        Args:
            state: RobotState

        Returns:
            RobotState
        """

        ...  # pragma no cover

    @abstractmethod
    def states(self) -> List[RobotState]:
        """ Gets a list of all the RobotStates in this collection. """

        ...

    def visualize(self, start: RobotState, goal: RobotState):
        """

        https://jakevdp.github.io/PythonDataScienceHandbook/04.12-three-dimensional-plotting.html

        """

        import numpy as np
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D

        ax = plt.axes(projection='3d')

        # Draw the vertices
        states = self.states()
        x= [state[0] for state in states]
        y = [state[1] for state in states]
        z = np.zeros(len(y))
        ax.scatter3D(x, y, z, s=1)

        # Highlight the start and goal states
        ax.scatter3D([start.configuration[0]], [start.configuration[1]], [0.], s=30)
        ax.scatter3D([goal.configuration[0]], [goal.configuration[1]], [0.], s=30)

        """ Draw the edges
        
        for state in states:
            # Draw the edges
            for other_state in self.edges[state]:
         
                x = [state.configuration[0], other_state.configuration[0]]
                y = [state.configuration[1], other_state.configuration[1]]
                z = np.zeros(2)

                ax.plot(x, y, z, color='b')
        """
        plt.show()

    @abstractmethod
    def add_edge(self, state1: RobotState, state2: RobotState):
        """ Adds edges between the provided RobotStates.

        Args:
            state1: RobotState
            state2: RobotState
        """

        ...  # pragma no cover

    def distance(self, state1: RobotState, state2: RobotState) -> float:
        """ Computes the distance between the two states.

         Args:
             state1: RobotState
             state2: RobotState

         Returns:
             float
                The distance.
         """

        return np.sqrt(sum_square_error(state1.configuration, state2.configuration))


class NaiveRobotStateCollection(RobotStateCollection):
    """ Stores a set of RobotStates as a flat list. """

    def __init__(self):
        """ Constructor. """

        self.states = []
        self.edges = {}  # RobotState -> List[RobotState]

    def size(self) -> int:
        """ Gets the number of RobotStates in this collection.

        Returns:
            int
        """

        if self.states is None:
            return 0
        return len(self.states)

    def insert(self, state: RobotState):
        """ Inserts a new RobotState into this collection.

        Args:
             state: RobotState
        """

        self.states.append(state)

    def nearest(self, input_state: RobotState) -> RobotState:
        """ Finds the nearest RobotState to the provided state.

        Args:
            input_state: RobotState

        Returns:
            RobotState
        """

        if self.size() == 0:
            return None

        # FIX ME - This could be much faster
        distances = [self.distance(input_state, state) for state in self.states]
        min_distance = min(distances)
        min_distance_index = distances.index(min_distance)
        return self.states[min_distance_index]

    def add_edge(self, state1: RobotState, state2: RobotState):
        """ Adds edges between the provided RobotStates.

        Args:
            state1: RobotState
            state2: RobotState
        """

        # Add edge from state1 to state2
        if state1 in self.edges:
            self.edges[state1].append(state2)
        else:
            self.edges[state1] = [state2]

        # Add edge from state2 to state1
        if state2 in self.edges:
            self.edges[state2].append(state1)
        else:
            self.edges[state2] = [state1]

    def distance_to_nearest(self, robot_state: RobotState) -> float:
        """ Computes the distance between this instance and its nearest neighbor in the collection.

         Args:
             robot_state: RobotState

         Returns:
             float
                The distance from this instance to its nearest neighbor in the collection.
         """

        nearest_state = self.nearest(robot_state)
        if nearest_state is None:
            return float('inf')
        return self.distance(nearest_state, robot_state)

    def empty(self):
        """ Empties or removes all RobotStates from the collection."""

        self.states = []
        self.edges = {}

    def states(self):
        return self.states()


class KDTree(RobotStateCollection):
    """ KDTree storage of a set of RobotStates. """

    def __init__(self, dimension: int):
        """ Constructor.

         Args:
             dimension: int
         """

        self.tree = kdtree.create(dimensions=dimension)
        self.dimension = dimension
        self._size = 0
        self.edges = {}

    def insert(self, state: RobotState):
        """ Inserts a new RobotState into this collection.

        Args:
            state: RobotState
        """

        self.tree.add(state.configuration_as_tuple)
        self._size += 1

    def nearest(self, state: RobotState) -> RobotState:
        """ Finds the nearest RobotState to the provided state.

        Args:
            state: RobotState

        Returns:
            RobotState
        """

        tuple = self.tree.search_nn(state.configuration_as_tuple)
        if tuple is None:
            return None

        input_configuration = tuple[0].data
        distance = tuple[1]
        configuration_array = np.array([num for num in input_configuration])
        return JointState(configuration_array)

    def add_edge(self, state1: RobotState, state2: RobotState):
        """ Adds edges between the provided RobotStates.

        Args:
            state1: RobotState
            state2: RobotState
        """

        # Add edge from state1 to state2
        if state1 in self.edges:
            self.edges[state1].append(state2)
        else:
            self.edges[state1] = [state2]

        # Add edge from state2 to state1
        if state2 in self.edges:
            self.edges[state2].append(state1)
        else:
            self.edges[state2] = [state1]

    def size(self) -> int:
        """ Gets the number of RobotStates in this collection.

        Returns:
            int
        """
        return self._size

    def empty(self):
        """ Empties or removes all RobotStates from the collection."""

        self.tree = kdtree.create(dimensions=self.dimension)
        self.edges = {}
        self._size = 0

    def distance_to_nearest(self, robot_state: RobotState) -> float:
        """ Computes the distance between this instance and its nearest neighbor in the collection.

         Args:
             robot_state: RobotState

         Returns:
             float
                The distance from this instance to its nearest neighbor in the collection.
         """

        nearest_state = self.nearest(robot_state)
        if nearest_state is None:
            return float('inf')
        return self.distance(nearest_state, robot_state)

    def states(self) -> List[RobotState]:
        """Gets a list of all the states in the KDTree.

        Returns:
            List[RobotState]
        """

        kd_nodes = list(kdtree.level_order(self.tree))
        return [node.data for node in kd_nodes]
