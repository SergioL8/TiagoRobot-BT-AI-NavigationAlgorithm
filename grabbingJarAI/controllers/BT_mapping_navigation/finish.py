import py_trees

# the objective of this behaviour is to keep the robot stopped forever
class Finish(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard): # initialize the class
        super(Finish, self).__init__(name)
        self.robot = blackboard.robot
        self.blackboard = blackboard
    
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep()) # # get the time step of the current world.

        # get motors from robot
        self.motor_left = self.robot.getDevice('wheel_left_joint')
        self.motor_right = self.robot.getDevice('wheel_right_joint')
        
        self.motor_left.setPosition(float('inf'))
        self.motor_right.setPosition(float('inf'))
    
    def update(self):

        # stop the robot indefinitely
        self.motor_left.setVelocity(0.0)
        self.motor_right.setVelocity(0.0)
        return py_trees.common.Status.RUNNING