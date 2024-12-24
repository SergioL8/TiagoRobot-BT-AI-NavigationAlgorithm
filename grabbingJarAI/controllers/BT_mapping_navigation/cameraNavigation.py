import py_trees
import numpy as np
from scipy.spatial.transform import Rotation as R




class CameraNavigation(py_trees.behaviour.Behaviour):

    def __init__ (self, name, blackboard): # initialize the class
        super(CameraNavigation, self).__init__(name)
        # load the robot and the blackboard from the blackboard
        self.robot = blackboard.robot
        self.blackboard = blackboard


    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep()) # get the time step of the current world.

        # Compass
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)

        # Camera
        self.camera = self.robot.getDevice("Astra rgb")
        self.camera.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)

        # Motors
        self.motor_left = self.robot.getDevice('wheel_left_joint')
        self.motor_right = self.robot.getDevice('wheel_right_joint')
        
        # Set motor position to infinity
        self.motor_left.setPosition(float('inf'))
        self.motor_right.setPosition(float('inf'))




    def update(self):
        print('Camera Navigation')
        
        MAX_SPEED = 5.00 

        alpha = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1]) # get orientation
        objects = self.camera.getRecognitionObjects() # get a list of objects that the camera recognizes
        object = None

        # find the red object
        for obj in objects:
            if obj.colors[0] == 0.55 and obj.colors[1] == 0.06 and obj.colors[2] == 0.06:
                object = obj
        
        # if no red object found, use the first object
        if object is None:
            object = objects[0]
            
        # if object still none, then no objects have been detected by the camera
        if object is None:
            print('Object not found')
            return py_trees.common.Status.FAILURE
        

        # get the position of the object with AI
        position = object.getPosition()
        dx = position[0]
        dy = position[1]

        # get the angle of the object with respect to the robot
        beta = np.arctan2(dy, dx)

        # get the error in alignment of the robot (-0.062 is the offset of the arm)
        alignmentError = beta - 0.062

        # making sure error stays within bounds
        if alignmentError > np.pi: 
            alignmentError = alignmentError-2*np.pi
            
        if alignmentError < -np.pi:
            alignmentError = alignmentError+2*np.pi

        # get the distance of the object from the robot
        rho = np.sqrt(object.getPosition()[0]**2 + object.getPosition()[2]**2)

        # if the object is close enough, stop the robot, we have reached the object
        if rho < 0.84:
            return py_trees.common.Status.SUCCESS


        p1 = 2.50 # angular velocity
        p2 = 1.25 # foward velocity (1.25)
        
        # compute the velocity of each wheel
        phildot = -alignmentError*p1 + rho*p2
        phirdot = alignmentError*p1 + rho*p2
        
        # check the velocity doesn't exceed the max velocity
        phildot = min(phildot, MAX_SPEED)
        phirdot = min(phirdot, MAX_SPEED)
        
        # set the spped of the motors
        self.motor_left.setVelocity(phildot)
        self.motor_right.setVelocity(phirdot)

        return py_trees.common.Status.RUNNING



    def terminate(self, new_status):

        # when finished stop the robot
        self.motor_left.setVelocity(0)
        self.motor_right.setVelocity(0)

        return super().terminate(new_status)