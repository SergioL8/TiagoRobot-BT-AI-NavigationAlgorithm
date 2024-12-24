import py_trees
import numpy as np




class Navigation(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Navigation, self).__init__(name)
        self.robot = blackboard.robot # load robot from the blackboard
        self.blackboard = blackboard # load the blackboard
    
    
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())# get the time step of the current world.
        # GPS
        self.gps =  self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        
        # Compass
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        
        # Motors
        self.motor_left = self.robot.getDevice('wheel_left_joint')
        self.motor_right = self.robot.getDevice('wheel_right_joint')
        
        # Set motor position to infinity
        self.motor_left.setPosition(float('inf'))
        self.motor_right.setPosition(float('inf'))
        
        self.marker = self.robot.getFromDef("marker").getField("translation") # get the pingpong ball object and update its position with marker

        self.logger.debug(" %s [Navigation::Setup()]" %self.name)

    def initialise(self):
        # Set motor position to infinity
        self.motor_left.setVelocity(0.0)
        self.motor_right.setVelocity(0.0)
        
        # variable declaration
        self.index = 0
        self.counter = 0
        
        self.logger.debug(" %s [Navigation::Initialise()]" %self.name)
        
        self.WP = self.blackboard.waypoints # load the waypoints
                
        
    def update(self):
    
        # variable declaration
        MAX_SPEED = 5.00 # set the max speed (really 10.2)
        
        # this if statement is only applicable when the robot is navigating to get the map
        if self.counter >= 8: # check if we have already completed the all the waypoints to move around the table
            print("Loop completed, you can find the convoled map in the same folder that contains controller")
            self.motor_left.setVelocity(0) # stop the robot
            self.motor_right.setVelocity(0)
            return py_trees.common.Status.SUCCESS # return success
    
    
          
        self.logger.debug(" %s [Navigation::Update()]" %self.name)
        
        # get values from sensors
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        alpha = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1]) # get orientation
        
        self.marker.setSFVec3f([*self.WP[self.index], 0.0]) # here we are setting the location of the ping pong ball (marker)
        
        rho = np.sqrt((xw - self.WP[self.index][0])**2 + (yw - self.WP[self.index][1])**2) # euclidian distance
        
        alphaE = np.arctan2(self.WP[self.index][1]-yw, self.WP[self.index][0]-xw) - alpha # angle error
        
        
        # making sure error stays within bounds
        if alphaE > np.pi: 
            alphaE = alphaE-2*np.pi
            
        if alphaE < -np.pi:
            alphaE = alphaE+2*np.pi
            
            
        # this if statement is used to switch between the logic of when the robot is mapping the environment and not
        if len(self.WP) < 10: # if the length of the waypoints is less than 10 then we are mapping the environment
            if rho<0.5: # udpate the marker
                if self.counter < 3: # if we have't gotten to the end of the first lap
                    self.index = self.index+1 # increase index
                    self.marker.setSFVec3f([*self.WP[self.index],0]) # update marker (direction)
                    self.counter += 1
                else: # we have already completed the first lap
                    self.index = self.index - 1 # substract index
                    self.marker.setSFVec3f([*self.WP[self.index],0]) # update maker (direction)
                    self.counter +=1
        else: # logic for when we are not looping through to scan the environment
            if rho<0.5: # udpate the marker
                if self.index+1 >= len(self.WP):
                    return py_trees.common.Status.SUCCESS
                self.index = self.index+1
                self.marker.setSFVec3f([*self.WP[self.index],0])
            
        
        p1 = 1.50 # angular velocity
        p2 = 1.25 # foward velocity
        
        # compute the velocity of each wheel
        phildot = -alphaE*p1 + rho*p2
        phirdot = alphaE*p1 + rho*p2
        
        # check the velocity doesn't exceed the max velocity
        phildot = min(phildot, MAX_SPEED)
        phirdot = min(phirdot, MAX_SPEED)
        
        
        # set the spped of the motors
        self.motor_left.setVelocity(phildot)
        self.motor_right.setVelocity(phirdot)
        
        
        
        return py_trees.common.Status.RUNNING
        
      