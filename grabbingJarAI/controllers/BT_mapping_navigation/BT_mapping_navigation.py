import py_trees
from py_trees.composites import Sequence, Parallel, Selector
from py_trees.blackboard import Blackboard
from controller import Robot, Supervisor
from os.path import exists
from navigation import Navigation
from mapping import Mapping
from planning import Planning
from armPosition import ArmPosition
from moveBack import MoveBack
from cameraNavigation import CameraNavigation
from gripper import Gripper
from finish import Finish
import numpy as np
import math
from scipy.spatial.transform import Rotation as R




blackboard = Blackboard() #initialize the blackboard of py_trees

robot = Supervisor()# create the Robot instance.
timestep = int(robot.getBasicTimeStep())# get the time step of the current world.
MAX_SPEED = 5.00 # set the max speed (really 10.2)

# position of robot when approaching a jar
defaultGrabPosition = {
    'torso_lift_joint' : 0.225,
    'arm_1_joint' : 1.65,
    'arm_2_joint' : 0.75,
    'arm_3_joint' : 0,
    'arm_4_joint' : 1.25,
    'arm_5_joint' : 1.5,
    'arm_6_joint' : 0.5,
    'arm_7_joint' : 0,
    'gripper_left_finger_joint' : 0.045,
    'gripper_right_finger_joint' : 0.045,
    'head_1_joint': 0,
    'head_2_joint': 0,}
    
# position of robot when approaching a jar but with grippers closed
grabGrippers = {
    'torso_lift_joint' : 0.325,
    'arm_1_joint' : 2.5,
    'arm_2_joint' : 0.75,
    'arm_3_joint' : 0,
    'arm_4_joint' : 1.25,
    'arm_5_joint' : 1.5,
    'arm_6_joint' : 0.5,
    'arm_7_joint' : 0,
    'gripper_left_finger_joint' : 0.01,
    'gripper_right_finger_joint' : 0.01,
    'head_1_joint': 0,
    'head_2_joint': 0,}
    

# position of robot with the arm closer to the center
# of the robot to move the center of mass closer to the center of the robot
movePosition = {
    'torso_lift_joint' : 0.225,
    'arm_1_joint' : 1.65,
    'arm_2_joint' : 0.75,
    'arm_3_joint' : 0,
    'arm_4_joint' : -0.32,
    'arm_5_joint' : 1.5,
    'arm_6_joint' : -1.39,
    'arm_7_joint' : 0,
    'gripper_left_finger_joint' : 0.01,
    'gripper_right_finger_joint' : 0.01,
    'head_1_joint': 0,
    'head_2_joint': 0,}


# position to drop the jar by lowering the body and getting closer to the table
dropPosition = {
    'torso_lift_joint' : 0.13,
    'arm_1_joint' : 1.65,
    'arm_2_joint' : 0.75,
    'arm_3_joint' : 0,
    'arm_4_joint' : 1.25,
    'arm_5_joint' : 1.5,
    'arm_6_joint' : 0.5,
    'arm_7_joint' : 0,
    'gripper_left_finger_joint' : 0.045,
    'gripper_right_finger_joint' : 0.045,
    'head_1_joint': 0,
    'head_2_joint': 0,}




blackboard.robot = robot # add the robot to the blackboard
blackboard.waypoints = [(0.35, -2.9), (-1.55, -2.9), (-1.5, 0.11), (0.4, -0.05)] # add the waypoints of getting the map to the blackboard



# this class is used to check if the file that contains the map exists
class DoesMapExist(py_trees.behaviour.Behaviour):
    def update(self):
        # check if the file exists
        file_exists = exists('cspace.npy') 
        if file_exists: # determine if the map exists of not and return success or failuer accordingly 
            print('Map already exists')
            return py_trees.common.Status.SUCCESS
        else:
            print('Map does not exists')
            return py_trees.common.Status.FAILURE
            
            

# declare the behaviour tree
tree = Sequence("Main", children=[
    
    # set the position of the robot to the approaching position with the gripper open
    Parallel(
        name="Setting default position",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
        children=[
            ArmPosition('startingPosition', blackboard, defaultGrabPosition),
            Gripper('closeGripper', blackboard, defaultGrabPosition, False),
        ]
    ),
    CameraNavigation('navigateFirstItem', blackboard), # use the camera to approach the first jar
    Gripper('grabringItem1', blackboard, grabGrippers, True), # grab the first time
    MoveBack('gettingSpace1', blackboard), # get space to perform the rotation without crashing
    ArmPosition('movingPosition1', blackboard, movePosition), # move the robot to the moving position
    Planning('planning1', blackboard, (-0.24, -1.07)), # plan the route to the table
    Navigation('movingItem1', blackboard), # move to the table
    ArmPosition('placingItem1', blackboard, dropPosition), # place the arm closer to the table
    Gripper('droppingItem1', blackboard, dropPosition, False), # drop the jar
    ArmPosition('returning1', blackboard, movePosition), # set the position of the robot to the moving position
    MoveBack('gettingSpace3', blackboard), # get space to perform the rotation
    Planning('planning2', blackboard, (0.83, -0.13)), # plan to the starting point
    Navigation('restartingLocation1', blackboard, turning=True), # navigate to the starting position
    ArmPosition('movingPosition3', blackboard, defaultGrabPosition), # set the arm to the default grabbing position
    
    # perform the same actions as above for jar number 2
    CameraNavigation('navigateSecondItem', blackboard),
    Gripper('grabringItem2', blackboard, grabGrippers, True),
    MoveBack('gettingSpace4', blackboard),
    ArmPosition('movingPosition4', blackboard, movePosition),
    Planning('planning3', blackboard, (-0.04, -0.34)), #(-0.17, -0.59)
    Navigation('movingItem2', blackboard),
    ArmPosition('placingItem2', blackboard, dropPosition),
    Gripper('droppingItem2', blackboard, dropPosition, False),
    ArmPosition('returning2', blackboard, movePosition),
    MoveBack('gettingSpace5', blackboard),
    Planning('planning4', blackboard, (1.2, 0.6)), # (1.2, 0.44)
    Navigation('restartingLocation2', blackboard, turning=True),
    ArmPosition('movingPosition5', blackboard, defaultGrabPosition),
    
    # perform the same actions as above for jar number 3
    CameraNavigation('navigateThirdItem', blackboard),
    Gripper('grabringItem3', blackboard, grabGrippers, True),
    MoveBack('gettingSpace5', blackboard),
    ArmPosition('movingPosition5', blackboard, movePosition),
    Planning('planning4', blackboard, (-0.05, -1.14)), #(0.18, -0.87)
    Navigation('movingItem3', blackboard),
    ArmPosition('placingItem3', blackboard, dropPosition),
    Gripper('droppingItem3', blackboard, dropPosition, False),
    
    # set the velocity of the robot to 0
    Finish('finish', blackboard),
    ], memory=True)


# setup the tree
tree.setup_with_descendants()


# execute the behaviour 
while robot.step(timestep) != -1: 
    
    # tick the tree
    tree.tick_once()
    
    