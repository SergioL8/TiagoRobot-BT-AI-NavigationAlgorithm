import py_trees
import numpy as np
from helperFunctions import *
from scipy import signal
from matplotlib import pyplot as plt


class Mapping(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Mapping, self).__init__(name)
        
        self.hasrun = False
        self.robot = blackboard.robot # load the robot from the balckboard
        
        
        
    def setup(self):
        
        self.timestep = int(self.robot.getBasicTimeStep())# get the time step of the current world.
        # GPS
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        
        # Compass
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        
        #lidar sensor
        self.lidar = self.robot.getDevice('Hokuyo URG-04LX-UG01')
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()
        
        # Display
        self.display = self.robot.getDevice('display')
        
        self.logger.debug(" %s [Mapping::Setup()]" %self.name)
        
        
        
    def initialise(self):

        self.logger.debug(" %s [Mapping::initialise()]" %self.name)
        
        self.map = np.zeros((300, 300)) # initialize map
        self.angles = np.linspace(np.radians(120), np.radians(-120), 667) # initialize the angles of the lidar sensor
        self.angles = self.angles[80:-80] # remove the angles that correspond to the angle overed by the box of the robot
        
        self.kernel = np.ones((44,44)) # set the kernnel used to convole the map
        
    def update(self):
    
        self.hasrun = True # set the has run variable to true
        
        # Retreive data from devices
        xw = self.gps.getValues()[0] # get coordinate x
        yw = self.gps.getValues()[1] # get coordinate y
        alpha = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1]) # get orientation

        self.logger.debug(" %s [Mapping::Values Retreived Successfully]" %self.name)
        ranges = np.array(self.lidar.getRangeImage()) # get lidar values
        ranges = ranges[80:-80] # remove the values covered by the box of the robot
        ranges[ranges == np.inf] = 100 # set all the lidar values that are infinity to 100
        
        
        # Perform all necessary computations for the lidar locations
        w_T_r = np.array([[np.cos(alpha), -np.sin(alpha), xw],
                          [np.sin(alpha), np.cos(alpha), yw],
                          [0, 0, 1]])
        
        LTR = np.array([[1, 0, 0.202],
                        [0, 1, 0],
                        [0, 0, 1]])
                        
        LTW = w_T_r @ LTR
        
        X_r = np.array([ranges*np.cos(self.angles),
                        ranges*np.sin(self.angles),
                        np.ones(len(self.angles))])
        D = LTW @ X_r


        # Draw the robot's track
        px, py = world2map(xw, yw)
        self.display.setColor(0xFF0000)  # Set color to red
        self.display.drawPixel(px, py)  # Draw the pixel at the calculated position
        
        
        # Logic for object probability
        for d in np.transpose(D):
            px, py = world2map(d[0], d[1]) # this is the value of the laser in the map
            self.map[px, py] = min(self.map[px, py] + 0.01, 1) # increment the probability of the object in the map 
            v=int(self.map[px,py]*255) # convert map probabilities into values from 0 to 255
            color=(v*256**2+v*256+v) # compute the gray color
            self.display.setColor(color)
            self.display.drawPixel(px, py)
        
        
        return py_trees.common.Status.RUNNING



    def terminate(self, new_status):
        self.logger.debug(" %s [Mapping::Terminate()]" %self.name)
        
        if self.hasrun:
            cmap = signal.convolve2d(self.map,self.kernel,mode='same') # convole the map
            cspace = cmap>0.9 # only include the objects that have a probability of existing >0.9
            np.save('cspace',cspace) # save the file
            plt.imshow(cspace) # save the figure of the convolved map
            plt.savefig("convolved_mapp.png")
            plt.close()
        
        
        
        