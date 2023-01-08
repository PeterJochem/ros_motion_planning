from typing import List
from .robot_state import RobotState


class Path:
    """ A set of kinematic states without any schedule.  """

    def __init__(self, states: List[RobotState]):
        """ Constructor.

        Args:
            states: List[RobotState]
        """
        
        self.robot_states = states 

    def to_ros(self):
        """ Converts to ROS type. """
        
        ...

    def __len__(self):

        if self.robot_states is None:
            return 0
        return len(self.robot_states)

    def visualize(self):
        """ """

        import numpy as np
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D

        ax = plt.axes(projection='3d')

        # Draw the vertices
        states = self.robot_states
        x = [state.configuration_as_tuple[0] for state in states]
        y = [state.configuration_as_tuple[1] for state in states]
        z = np.zeros(len(y))
        ax.scatter3D(x, y, z, s=20)

        # Plot the edges
        for i in range(1, len(states)):
            start = states[i - 1]
            end = states[i]

            x = [start.configuration[0], end.configuration[0]]
            y = [start.configuration[1], end.configuration[1]]
            z = np.zeros(len(y))
            ax.plot3D(x, y, z)

        plt.show()
