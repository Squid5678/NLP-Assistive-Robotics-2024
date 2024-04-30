#!/usr/bin/env python

import rospy
import tf2_ros
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from gazebo_ros_link_attacher.srv import Attach, AttachRequest

import actionlib
import math
from enum import Enum

class Joints(Enum):
    joint_lift = 0
    joint_arm_l3 = 1
    joint_arm_l2 = 2
    joint_arm_l1 = 3
    joint_arm_l0 = 4
    joint_head_pan = 5
    joint_head_tilt = 6
    joint_wrist_yaw = 7
    joint_gripper_finger_left = 8
    joint_gripper_finger_right = 9
    translate_mobile_base = "translate_mobile_base"
    rotate_mobile_base = "rotate_mobile_base"

class JointController(object):
    MIN_LIFT = 0.3
    MAX_LIFT = 1.09
    MIN_WRIST_EXTENSION = 0.01
    MAX_WRIST_EXTENSION = 0.5
    MAX_GRIPPER_OPEN = 50 / 180 * math.pi
    MAX_GRIPPER_CLOSURE = -100 / 180 * math.pi

    def __init__(self):
        self.joint_states = JointState()

        # Arm controller
        self.arm_trajectory_client = actionlib.SimpleActionClient('/stretch_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.arm_trajectory_client.wait_for_server()

        # Gripper controller
        self.gripper_trajectory_client = actionlib.SimpleActionClient('/stretch_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.gripper_trajectory_client.wait_for_server()

        # Head controller
        self.head_trajectory_client = actionlib.SimpleActionClient('/stretch_head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.head_trajectory_client.wait_for_server()

        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)

    def joint_states_callback(self, data):
        self.joint_states = data

    def place(self):
        self.set_cmd(joints=[Joints.gripper_aperture], values=[0.0445], wait=True)

    def retract_arm(self):
        self.set_cmd(joints=[Joints.wrist_extension], values=[JointController.MIN_WRIST_EXTENSION], wait=True)

    def stow(self):
        # set arm to stow mode
        self.set_cmd(joints=[
            Joints.joint_wrist_yaw,
            Joints.joint_arm_l0,
            Joints.joint_arm_l1,
            Joints.joint_arm_l2,
            Joints.joint_arm_l3,
            Joints.joint_lift
            ],
            values=[
                math.pi, 
                JointController.MIN_WRIST_EXTENSION/4, 
                JointController.MIN_WRIST_EXTENSION/4, 
                JointController.MIN_WRIST_EXTENSION/4, 
                JointController.MIN_WRIST_EXTENSION/4, 
                JointController.MIN_LIFT
            ],
            wait=True)

        # set gripper to stow mode
        self.set_cmd(joints=[
            Joints.joint_gripper_finger_left,
            Joints.joint_gripper_finger_right,
            ],
            values=[0, 0], 
            wait=True) # gripper close
        
        # set head to stow mode
        self.set_cmd(joints=[
            Joints.joint_head_pan,
            Joints.joint_head_tilt,
            ],
            values=[
                0, 
                -math.pi / 6, 
            ], 
            wait=True) # camera facing forward, camera horizontal to floor
    
    def set_cmd(self, joints, values, wait):
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(5.0)
        point.positions = values

        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.goal_time_tolerance = rospy.Time(5.0)

        joint_names = []

        for joint in joints:
            joint_names.append(Joints(joint).name)

        trajectory_goal.trajectory.joint_names = joint_names

        trajectory_goal.trajectory.points = [point]
        trajectory_goal.trajectory.header.stamp = rospy.Time.now()
        
        # send control command to gripper
        if "gripper" in joint_names[0]:
            self.gripper_trajectory_client.send_goal(trajectory_goal)
        # send control command to head 
        elif "head" in joint_names[0]:
            self.head_trajectory_client.send_goal(trajectory_goal)
        # send control command to arm
        else:
            self.arm_trajectory_client.send_goal(trajectory_goal)

        if (wait):
            if "gripper" in joint_names[0]:
                rospy.sleep(rospy.Duration(2))
            elif "head" in joint_names[0]:
                self.head_trajectory_client.wait_for_result(rospy.Duration(5.0))
            else:
                self.arm_trajectory_client.wait_for_result(rospy.Duration(5.0))

class ArmController(object):
    def get_bounded_lift(self, lift_value):
        if lift_value > JointController.MAX_LIFT:
            return JointController.MAX_LIFT
        elif lift_value < JointController.MIN_LIFT:
            return JointController.MIN_LIFT

        return lift_value

    def get_bounded_extension(self, extension_value):
        if extension_value > JointController.MAX_WRIST_EXTENSION:
            return JointController.MAX_WRIST_EXTENSION
        elif extension_value < JointController.MIN_WRIST_EXTENSION:
            return JointController.MIN_WRIST_EXTENSION

        return extension_value

    def __init__(self):
        self.rate = 10
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.joint_controller = JointController()

        rospy.wait_for_service('/link_attacher_node/attach')
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        rospy.wait_for_service('/link_attacher_node/detach')
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        
        # Arm controller
        self.arm_trajectory_client = actionlib.SimpleActionClient(
            '/stretch_arm_controller/follow_joint_trajectory', 
            FollowJointTrajectoryAction
        )
        self.arm_trajectory_client.wait_for_server()

        # Gripper controller
        self.gripper_trajectory_client = actionlib.SimpleActionClient(
            '/stretch_gripper_controller/follow_joint_trajectory', 
            FollowJointTrajectoryAction
        )
        self.gripper_trajectory_client.wait_for_server()

        # Head controller
        self.head_trajectory_client = actionlib.SimpleActionClient(
            '/stretch_head_controller/follow_joint_trajectory', 
            FollowJointTrajectoryAction
        )
        self.head_trajectory_client.wait_for_server()

        # set arm to stow mode
        self.joint_controller.stow()
    
    def joint_movement(self):
        stop = 0
        while not stop:
            joint_type = input(
                "Please choose what type of manipulator movement, 0. arm extension, 1. arm lift, 2. wrist rotation, 3. gripper openness, 4. rotate the head, 5. tilt the head, 6. grab the object, 7. place the object:\n"
            )
            joint_type = int(joint_type)
            
            if joint_type == 0:
                extension = input("Input the desired extension of the arm (e.g., 0.3), maximal extension is 0.5:\n")
                extension = float(extension)
                self.joint_controller.set_cmd(joints=[
                    Joints.joint_arm_l0,
                    Joints.joint_arm_l1,
                    Joints.joint_arm_l2,
                    Joints.joint_arm_l3,
                    ],
                    values=[
                        extension/4,
                        extension/4,
                        extension/4,
                        extension/4,
                    ],
                    wait=True)
            elif joint_type == 1:
                lift = input("Input the desired height of the arm, e.g., 0.5, maximal lift is 1.09:\n")
                lift = float(lift)
                self.joint_controller.set_cmd(joints=[
                    Joints.joint_lift,
                    ],
                    values=[lift],
                    wait=True)
            elif joint_type == 2:
                rotation = input("Input the desired input angle in radius:\n")
                rotation = float(rotation)
                self.joint_controller.set_cmd(joints=[
                    Joints.joint_wrist_yaw,
                    ],
                    values=[rotation],
                    wait=True)
            elif joint_type == 3:
                rotation = input("Input the desired gripper rotation, e.g., 0.87 for maximal open and  -1.74 for maximal close:\n")
                rotation = float(rotation)
                self.joint_controller.set_cmd(joints=[
                    Joints.joint_gripper_finger_left,
                    Joints.joint_gripper_finger_right,
                    ],
                    values=[rotation, rotation],
                    wait=True)
            elif joint_type == 4:
                rotation = input("Input the desired rotation of head:\n")
                rotation = float(rotation)
                self.joint_controller.set_cmd(joints=[
                    Joints.joint_head_pan,
                    ],
                    values=[rotation],
                    wait=True)
            elif joint_type == 5:
                rotation = input("Input the desired tilt rotation:\n")
                rotation = float(rotation)
                self.joint_controller.set_cmd(joints=[
                    Joints.joint_head_tilt,
                    ],
                    values=[rotation],
                    wait=True)
            elif joint_type == 6:
                req = AttachRequest()
                req.model_name_1 = 'robot'
                req.link_name_1 = 'link_gripper_finger_left'
                req.model_name_2 = 'coke_can'
                req.link_name_2 = 'link'
                res = self.attach_srv.call(req)
            elif joint_type == 7:
                req = AttachRequest()
                req.model_name_1 = 'robot'
                req.link_name_1 = 'link_gripper_finger_left'
                req.model_name_2 = 'coke_can'
                req.link_name_2 = 'link'
                res = self.detach_srv.call(req)

            stop = input("Do you wanna continue sending command to control the arm? 0. Yes, 1. No\n")
            stop = int(stop)

        return True

if __name__ == '__main__':
    rospy.init_node('arm_controller')
    arm_controller = ArmController()
    arm_controller.joint_movement()
