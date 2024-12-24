import py_trees
import numpy as np
import math


# this class mission is to set the positino of the arm and body 
class ArmPosition(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, robot_joints): # initialize the class
        super(ArmPosition, self).__init__(name)
        self.robot = blackboard.robot # load robot from the blackboard
        self.blackboard = blackboard # load the blackboard
        self.robot_joints = robot_joints # get the objective position of the robot
        self.sensors = {}  # Dictionary to store the sensors
        self.name = name


    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep()) # get the time step of the current world.

        # Motors
        self.motor_left = self.robot.getDevice('wheel_left_joint')
        self.motor_right = self.robot.getDevice('wheel_right_joint')
        
        # Set motor position to infinity
        self.motor_left.setPosition(float('inf'))
        self.motor_right.setPosition(float('inf'))

        # Loop through the robot joints and set the position
        for joint_name, target_position in self.robot_joints.items():

            # skip the gripper joints since they are managed by another behaviour
            if joint_name == 'gripper_left_finger_joint' or joint_name == 'gripper_right_finger_joint':
                continue

            motor = self.robot.getDevice(joint_name)  # Get the motor handle for the joint
            motor.setPosition(target_position)  # Set the motor to the desired position
            sensor = self.robot.getDevice(f"{joint_name}_sensor")

            # Enable the position sensor for this motor
            sensor.enable(self.timestep)
            self.sensors[joint_name] = sensor
        
        

    def initialise(self):

        # Set motor velocity to 0
        self.motor_left.setVelocity(0.0)
        self.motor_right.setVelocity(0.0)

        # Loop through the robot joints and set the position
        for joint_name, target_position in self.robot_joints.items():

            # Loop through the robot joints and set the position
            if joint_name == 'gripper_left_finger_joint' or joint_name == 'gripper_right_finger_joint':
                continue

            motor = self.robot.getDevice(joint_name)  # Get the motor handle for the joint
            motor.setPosition(target_position)  # Set the motor to the desired position
            sensor = self.robot.getDevice(f"{joint_name}_sensor")

            # Enable the position sensor for this motor
            sensor.enable(self.timestep)
            self.sensors[joint_name] = sensor




    def update(self):
        print('Arm Position')

        inPosition = True

        # Loop through the robot joints and check if the robot is in position
        for joint_name, target_position in self.sensors.items():
            
            # skip the gripper joints since they are managed by another behaviour
            if (joint_name == 'gripper_left_finger_joint' or joint_name == 'gripper_right_finger_joint'):
                continue

            # compute the difference between the target and current position
            targetMinusCurrent = target_position.getValue() -  self.robot_joints[joint_name]
            if targetMinusCurrent > 0.001 or targetMinusCurrent < -0.001:
                inPosition = False

        # return success if the robot is in position or running if not
        if inPosition:
            print('In correct position')
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING
            