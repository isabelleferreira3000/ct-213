import random
import math
from constants import *


class FiniteStateMachine(object):
    """
    A finite state machine.
    """
    def __init__(self, state):
        self.state = state

    def change_state(self, new_state):
        self.state = new_state

    def update(self, agent):
        self.state.check_transition(agent, self)
        self.state.execute(agent)


class State(object):
    """
    Abstract state class.
    """
    def __init__(self, state_name):
        """
        Creates a state.

        :param state_name: the name of the state.
        :type state_name: str
        """
        self.state_name = state_name

    def check_transition(self, agent, fsm):
        """
        Checks conditions and execute a state transition if needed.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent):
        """
        Executes the state logic.

        :param agent: the agent where this state is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")


class MoveForwardState(State):
    def __init__(self):
        super().__init__("MoveForward")
        # Todo: add initialization code
        print("MOVE FORWARD STATE")
        self.count = 0

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition
        time = self.count*SAMPLE_TIME

        if agent.get_bumper_state():
            go_back_state = GoBackState()
            state_machine.change_state(go_back_state)

        elif time > MOVE_FORWARD_TIME:
            move_in_spiral_state = MoveInSpiralState()
            state_machine.change_state(move_in_spiral_state)

    def execute(self, agent):
        # Todo: add execution logic
        self.count += 1
        agent.set_velocity(FORWARD_SPEED, 0)
        

class MoveInSpiralState(State):
    def __init__(self):
        super().__init__("MoveInSpiral")
        # Todo: add initialization code
        self.count = 0
        print("MOVE IN SPIRAL STATE")

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition
        time = self.count * SAMPLE_TIME

        if agent.get_bumper_state():
            go_back_state = GoBackState()
            state_machine.change_state(go_back_state)

        elif time > MOVE_IN_SPIRAL_TIME:
            move_forward_state = MoveForwardState()
            state_machine.change_state(move_forward_state)
        pass

    def execute(self, agent):
        # Todo: add execution logic
        self.count += 1
        time = self.count * SAMPLE_TIME
        agent.set_velocity(FORWARD_SPEED, FORWARD_SPEED/(INITIAL_RADIUS_SPIRAL+SPIRAL_FACTOR*time))
        
        pass


class GoBackState(State):
    def __init__(self):
        super().__init__("GoBack")
        # Todo: add initialization code
        self.count = 0
        print("GO BACK STATE")

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition
        time = self.count * SAMPLE_TIME

        if time > GO_BACK_TIME:
            rotate_state = RotateState()
            state_machine.change_state(rotate_state)
        pass

    def execute(self, agent):
        # Todo: add execution logic
        self.count += 1
        agent.set_velocity(BACKWARD_SPEED, 0)
        
        pass


class RotateState(State):
    def __init__(self):
        super().__init__("Rotate")
        # Todo: add initialization code
        self.count = 0
        self.angle = random.uniform(-math.pi, math.pi)
        print("random angle: " + str(self.angle*180/math.pi))
        print("ROTATE STATE")

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition
        time = self.count*SAMPLE_TIME
        ROTATE_TIME = abs(self.angle)/ANGULAR_SPEED
        if time > ROTATE_TIME:
            move_forward_state = MoveForwardState()
            state_machine.change_state(move_forward_state)
        pass
    
    def execute(self, agent):
        # Todo: add execution logic
        self.count += 1
        if self.angle > 0:
            agent.set_velocity(0, ANGULAR_SPEED)
        else:
            agent.set_velocity(0, -ANGULAR_SPEED)
        
        pass
