from typing import List, Tuple
from .robot_state import RobotState
from .time_scaler import TimeScaler
from .path import Path


class Trajectory:
    """ A path and a schedule. """

    def __init__(self, path: Path):
        """ Constructor.

        Args:
            path: Path
            time_scaler: TimeScaler
        """
        
        self.path = path
        self.trajectory = None

    def to_ros(self):
        """ Converts to ROS type.

            Note: This requires the joint names
            http://docs.ros.org/en/api/trajectory_msgs/html/msg/JointTrajectory.html
        """
        
        ...
