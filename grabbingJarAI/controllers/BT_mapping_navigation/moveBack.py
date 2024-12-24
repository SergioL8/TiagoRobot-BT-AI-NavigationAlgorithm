import py_trees
import time

class MoveBack(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(MoveBack, self).__init__(name)
        self.robot = blackboard.robot
        self.blackboard = blackboard

    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep()) # get the time step of the current world.

        # get motors
        self.motor_left = self.robot.getDevice('wheel_left_joint')
        self.motor_right = self.robot.getDevice('wheel_right_joint')
        
        self.motor_left.setPosition(float('inf'))
        self.motor_right.setPosition(float('inf'))

        # Calculate the duration to move back 4 meters (not really 4 meters)
        MAX_SPEED = 5.00  # meters per second
        distance = 4.0  # meters
        self.duration = distance / MAX_SPEED  # seconds
    
    def initialise(self):
        self.start_time = time.time()

    def update(self):
        print('Move Back')
        current_time = time.time() # get current time to compute the time the robot is moving back
        elapsed_time = current_time - self.start_time # compute the current time

        if elapsed_time < self.duration: # move back until the wanted duration has elapsed
            MAX_SPEED = 5.00
            self.motor_left.setVelocity(-MAX_SPEED)
            self.motor_right.setVelocity(-MAX_SPEED)
            return py_trees.common.Status.RUNNING
        else:
            self.motor_left.setVelocity(0)
            self.motor_right.setVelocity(0)
            return py_trees.common.Status.SUCCESS