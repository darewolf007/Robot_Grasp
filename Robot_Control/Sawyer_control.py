from Base_control import Base_control
from Utils.connect_ros.Robot_connect_by_ros1 import Sawyer_connect_ros1
from intera_interface import CHECK_VERSION
from Utils.python3_logging.python3_logging import init_logging
import intera_interface
import logging

class Sawyer_control(Base_control):
    def __init__(self):
        super().__init__()
        #TODO add argparse to these param
        self.robot_name = "Sawyer"
        self.arm_name = "right"
        self.ros_node_name = "intera_grasp"
        self.log_file_path = "./../tmp/log"
        self.joint_name_key = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2']
        self.ros_node = Sawyer_connect_ros1(self.arm_name)
        init_logging(self.robot_name, self.log_file_path)

    def init_ros_node(self):
        self.ros_node.ros1_init_ros_node(self.ros_node_name)

    def enable_robot(self):
        self.robot = intera_interface.RobotEnable()
        self.robot.enable()
        logging.info('successful enable robot !!!!!')

    def init_arm(self):
        init_arm = intera_interface.Limb(self.arm_name)
        self.set_init_position(init_arm.endpoint_pose()['position']) #TODO test what is endpoint joint or end-effector
        logging.info('successful initialize arm !!!!!')
        logging.info('end effector position is ' + str(self.get_init_position()))
        joint_angles = init_arm.joint_angles()
        self.set_init_joint_angles(joint_angles)

    def arm_inverse_kinematics(self, position, orient):
        return self.ros_node.ros1_inverse_kinematics(position, orient)

    def open_robot_gripper(self, interval_time = 0.2):
        gripper = intera_interface.Gripper(self.arm_name, CHECK_VERSION)
        self.ros_node.ros1_robot_sleep(interval_time)
        gripper.open()
        self.ros_node.ros1_init_ros_node(interval_time)

    def close_robot_gripper(self, interval_time = 0.2):
        gripper = intera_interface.Gripper(self.arm_name, CHECK_VERSION)
        self.ros_node.ros1_robot_sleep(interval_time)
        gripper.close()
        self.ros_node.ros1_init_ros_node(interval_time)

    def move_robot_to_point(self, position, orient, velocity):
        limb_handle = intera_interface.Limb(self.arm_name)
        limb_handle.set_joint_position_speed(velocity)
        limb_joints = self.arm_inverse_kinematics(position, orient)
        if limb_joints == -1:
            return False
        limb_handle.move_to_joint_positions(limb_joints)
        return True

    def move_robot_to_joint_positions(self, angles):
        limb_handle = intera_interface.Limb(self.arm_name)
        angles = dict(zip(limb_handle.joint_names(), angles))
        flag = limb_handle.move_to_joint_positions(angles)
