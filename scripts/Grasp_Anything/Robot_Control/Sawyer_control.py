from .Base_control import Base_control
from Grasp_Anything.utils import Robot_connect_by_ros1
from intera_interface import CHECK_VERSION
import intera_interface
# from Grasp_Anything.utils.python_function import init_logging
import logging
import numpy as np

class Sawyer_control(Base_control):
    def __init__(self):
        # super().__init__()
        #TODO add argparse to these param
        self.joint_name = None
        self.arm_cuff = None
        self.robot_name = "Sawyer"
        self.arm_name = "right"
        self.gripper_name = self.arm_name + "_gripper"
        self.ros_node_name = "intera_grasp"
        self.log_file_path = "./"
        self.trajectory_path = "./"
        self.trajectory_name = "Sawyer_traj"
        self.ros_node = Robot_connect_by_ros1.Sawyer_connect_ros1(self.arm_name)
        # init_logging(self.robot_name, self.log_file_path)

    def init_ros_node(self):
        # self.ros_node.ros1_init_ros_node(self.ros_node_name)
        logging.info('successful init ros node, node name is {}'.format(self.ros_node_name))

    def enable_robot(self):
        self.robot = intera_interface.RobotEnable()
        self.robot.enable()
        logging.info('successful enable robot !!!!!')

    def init_arm(self):
        self.limb_handle = intera_interface.Limb(self.arm_name)
        self.set_init_position(self.limb_handle.endpoint_pose()['position']) #TODO test what is endpoint joint or end-effector
        logging.info('successful initialize arm !!!!!')
        logging.info('end effector init-position is {}'.format(self.get_init_position()))
        joint_angles = self.limb_handle.joint_angles()
        self.set_init_joint_angles(joint_angles)
        logging.info('joint init-angles is {}'.format(self.get_init_joint_angles()))
        self.joint_name = self.limb_handle.joint_names()

    def init_gripper(self):
        try:
            self.arm_gripper = intera_interface.Gripper(self.gripper_name, CHECK_VERSION)
            logging.info("Electric gripper detected.")
        except Exception as e:
            self.arm_gripper = None
            logging.info("No electric gripper detected.")

    def arm_inverse_kinematics(self, position, orient):
        limb_joints = self.ros_node.ros1_inverse_kinematics(position, orient)
        if limb_joints == 1:
            logging.info("Service call failed")
        elif limb_joints == -1:
            logging.info("INVALID POSE - No Valid Joint Solution Found.")
        else:
            return True, limb_joints
        return False, limb_joints

    def open_robot_gripper(self, interval_time = 0.2):
        self.ros_node.ros1_robot_sleep(interval_time)
        self.arm_gripper.open()
        self.ros_node.ros1_robot_sleep(interval_time)

    def close_robot_gripper(self, interval_time = 0.2):
        self.ros_node.ros1_robot_sleep(interval_time)
        self.arm_gripper.close()
        self.ros_node.ros1_robot_sleep(interval_time)

    def move_robot_to_point(self, position, orient, velocity=0.5):
        self.limb_handle.set_joint_position_speed(velocity)
        success_check, limb_joints = self.arm_inverse_kinematics(position, orient)
        if success_check is True:
            self.limb_handle.move_to_joint_positions(limb_joints)
            # logging.info("success move robot to [ {} ]".format(self.))
            return True
        else:
            return False

    # TODO test1 and add try except
    def run(self, offset = np.array([0, 0 ,0])):
        self.receive_scene_info()
        self.set_grasp_offset(offset)
        self.run_grasp()
        self.putdown_object()

    def receive_scene_info(self):
        position, orient = self.ros_node.ros1_receive_grasp_infor()
        position_test = [position.x, position.y, position.z]
        orient_test =[orient.x, orient.y, orient.z, orient.w]
        self.set_grasp_param(np.array(position_test), np.array(orient_test))

    def putdown_object(self, putdown_orient = [0,1,0,0]):
        self.move_robot_to_point(self.get_putdown_position(), putdown_orient)
        self.open_robot_gripper()

    def move_robot_to_joint_angles(self, angles):
        joint_angles = dict(zip(self.limb_handle.joint_names(), angles))
        self.limb_handle.move_to_joint_positions(joint_angles)

    def set_grasp_param(self, grasp_center, grasp_orient):
        self.set_grasp_center(grasp_center)
        self.set_grasp_orient(grasp_orient)

    def run_grasp(self):
        grasp_position = self.get_grasp_center() + self.get_grasp_offset()
        grasp_orient = self.get_grasp_orient()
        success_check = self.move_robot_to_point(position=grasp_position, orient=grasp_orient)
        if success_check is True:
            self.close_robot_gripper()
            logging.info("grasp success")
        else:
            logging.info("grasp fail")

    def record_trajectory(self):
        trajectory_record = self.trajectory_path + self.trajectory_name
        self.arm_cuff = intera_interface.Cuff(self.arm_name)
        if trajectory_record:
            joints_right = self.limb_handle.joint_names()
            with open(trajectory_record, 'w') as f:
                f.write('time,')
                temp_str = '' if self.arm_gripper else '\n'
                f.write(','.join([j for j in self.joint_name]) + ',' + temp_str)
                if self.arm_gripper:
                    f.write(self.gripper_name + '\n')
                while not self.ros_node.ros1_done():
                    if self.arm_gripper:
                        if self.arm_cuff.upper_button():
                            self.arm_gripper.open()
                        elif self.arm_cuff.lower_button():
                            self.arm_gripper.close()
                    angles_right = self.get_joint_angles()
                    f.write("%f," % (self.ros_node.ros1_time_stamp(),))
                    f.write(','.join([str(x) for x in angles_right]) + ',' + temp_str)
                    if self.arm_gripper:
                        f.write(str(self.arm_gripper.get_end_effector_position()) + '\n')
                    self.ros_node.ros1_rate().sleep()

    def get_joint_angles(self):
        return [self.limb_handle.joint_angle(j) for j in self.joint_name]

    def get_end_effector_position(self):
        return self.arm_gripper.get_position()

    def set_gripper_velocity(self, speed):
        self.arm_gripper.set_cmd_velocity(speed)

    def get_gripper_velocity(self):
        return self.arm_gripper.get_cmd_velocity()

    def set_joint_velocities(self, speed):
        if len(speed) == len(self.joint_name):
            joint_speed = dict([(joint, speed) for i, joint in enumerate(self.joint_name)])
            self.limb_handle.set_joint_position_speed(joint_speed)
        # speed control can get speed??

    def move_to_neutral(self):
        self.limb_handle.move_to_neutral()
