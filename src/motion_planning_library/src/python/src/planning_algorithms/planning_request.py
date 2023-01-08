from .robot_state import RobotState


class PlanningRequest:
    """ Represents an input to a planning algorithm. """

    def __init__(self, start_state: RobotState, goal_state: RobotState):
        """ Constructor.

        Args:
            start_state: RobotState
            goal_state: RobotState
        """

        if start_state is None or goal_state is None:
            raise RuntimeError("Illegal input to planning request.")

        self._start_state = start_state
        self._goal_state = goal_state

        # constraints?
        # other parameters for the algorithm


    @property
    def start_state(self) -> RobotState:
        """ Gets this instance's start_state. """

        if self._start_state is None:
            raise RuntimeError("This PlanningRequest's start state is None.")
        return self._start_state

    @property
    def goal_state(self) -> RobotState:
        """ Gets this instance's goal_state. """

        if self._goal_state is None:
            raise RuntimeError("This PlanningRequest's goal state is None.")
        return self._goal_state


