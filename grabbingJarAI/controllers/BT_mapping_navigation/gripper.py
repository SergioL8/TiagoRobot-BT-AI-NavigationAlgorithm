import py_trees
import math


class Gripper(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, robot_joints, closing):
        super(Gripper, self).__init__(name)
        self.robot = blackboard.robot
        self.blackboard = blackboard
        self.closing = closing
        self.robot_joints = robot_joints
    
    def setup(self): # called only once for all behaviours
        self.timestep = int(self.robot.getBasicTimeStep())

        # get motor handles
        self.gripper_left = self.robot.getDevice('gripper_left_finger_joint')
        self.gripper_right = self.robot.getDevice('gripper_right_finger_joint')

        # set sensor handles
        self.sensor_left = self.robot.getDevice('gripper_left_sensor_finger_joint')
        self.sensor_right = self.robot.getDevice('gripper_right_sensor_finger_joint')

        # enable sensors
        self.sensor_left.enable(self.timestep)
        self.sensor_right.enable(self.timestep)

        # enable force feedback on the motors
        self.gripper_left.enableForceFeedback(self.timestep)
        self.gripper_right.enableForceFeedback(self.timestep)
    
    def initialise(self): # called for each behaviour 
        # set motor position
        self.gripper_left.setPosition(self.robot_joints['gripper_left_finger_joint'])
        self.gripper_right.setPosition(self.robot_joints['gripper_right_finger_joint'])
        

    def update(self):
        print('Gripper')
        inPosition = True

        # loop through the robot joints but only consider the gripper joints
        for joint_name, target_position in self.robot_joints.items():
            if joint_name == 'gripper_left_finger_joint' or joint_name == 'gripper_right_finger_joint':
                if joint_name == 'gripper_left_finger_joint':
                    targetMinusCurrent = target_position - self.sensor_left.getValue()
                else:
                    targetMinusCurrent = target_position - self.sensor_right.getValue()

                # check if the gripper is in position
                if targetMinusCurrent > 0.001 or targetMinusCurrent < -0.001:
                    inPosition = False

        # check if the gripper is closing to manage the force feedback
        if self.closing:

            # get teh force feedback from the gripper motors
            leftGripperForce = self.gripper_left.getForceFeedback()
            rightGripperForce = self.gripper_right.getForceFeedback()

            # if the force feedback is greater than -10, then the gripper is holding an object
            if leftGripperForce < -10 or rightGripperForce < -10:
                self.gripper_left.setPosition(math.floor(self.sensor_left.getValue()))
                self.gripper_right.setPosition(math.floor(self.sensor_right.getValue()))

                return py_trees.common.Status.SUCCESS
            
        
        if inPosition:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING



