class Base_control(object):
    def __init__(self):
        # robot name
        self.robot = None
        self.robot_name = None
        self.arm_name = None
        self.ros_node_name = None
        self.joint_name_key = None
        # robot grasp param
        self.__grasp_point = None
        self.__grasp_center = None
        self.__grasp_orient = None
        self.__grasp_offset = None
        # robot move param
        self.__init_position = None
        self.__end_position = None
        self.__put_down_position = None
        self.__arm_velocity = None
        # robot state param
        self.__init_joint_angles = None
        self.__joint_angles = None
        self.__joint_velocities = None

    # robot param set and get
    def set_init_position(self, init_position):
        self.__init_position = init_position

    def set_grasp_center(self, grasp_center):
        self.__grasp_center = grasp_center

    def set_grasp_point(self, grasp_point):
        self.__grasp_point = grasp_point

    def set_grasp_orient(self, grasp_orient):
        self.__grasp_orient = grasp_orient

    def set_grasp_offset(self, grasp_offset):
        self.__grasp_offset = grasp_offset

    def set_arm_velocity(self, arm_velocity):
        self.__arm_velocity = arm_velocity

    def set_init_joint_angles(self, init_joint_angles):
        self.__init_joint_angles = init_joint_angles

    def set_joint_angles(self, joint_angles):
        self.__joint_angles = joint_angles

    def set_joint_velocities(self, joint_velocities):
        self.__joint_velocities = joint_velocities

    def get_init_position(self):
        return self.__init_position

    def get_grasp_center(self):
        return self.__grasp_center

    def get_grasp_point(self):
        return self.__grasp_point

    def get_grasp_orient(self):
        return self.__grasp_orient

    def get_grasp_offset(self):
        return self.__grasp_offset

    def get_arm_velocity(self):
        return self.__arm_velocity

    def get_init_joint_angles(self):
        return self.__init_joint_angles

    def get_joint_angles(self):
        return self.__joint_angles

    def get_joint_velocities(self):
        return self.__joint_velocities

    # robot enable and initialize
    def enable_robot(self):
        pass

    def enable_arm(self):
        pass

    def init_arm(self):
        pass

    def init_ros_node(self):
        pass

    def init_robot(self):
        self.init_ros_node()
        self.enable_robot()
        self.enable_arm()
        self.init_arm()
        self.open_robot_gripper()

    # record robot state
    def get_end_position(self):
        pass

    def get_end_velocity(self):
        pass

    def get_end_orient(self):
        pass

    def get_joint_position(self):
        pass

    def get_joint_velocity(self):
        pass

    def get_joint_orient(self):
        pass

    def record_trajectory(self):
        pass

    # robot control
    def move_robot_forward(self):
        pass

    def move_robot_back(self):
        pass

    def open_robot_gripper(self, interval_time = 0.2):
        pass

    def close_robot_gripper(self, interval_time = 0.2):
        pass

    def receive_scene_info(self):
        pass

    def arm_inverse_kinematics(self, position, orient):
        pass

    def move_robot_to_point(self, position, orient, velocity):
        pass

    def move_robot_to_joint_positions(self, angles):
        pass

    def reset(self):
        pass

    def run(self):
        pass
