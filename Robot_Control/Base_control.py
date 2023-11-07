class Base_control(object):
    def __init__(self):
        # robot name
        self.robot = None
        self.robot_name = None
        self.arm_name = None
        self.ros_node_name = None
        # robot grasp param
        self.__grasp_center = None
        self.__grasp_orient = None
        self.__grasp_offset = None
        self.__put_down_position = None
        self.__grasp_velocity = None
        # robot state param
        self.__end_position = None
        self.__init_position = None
        self.__init_joint_angles = None
        self.__joint_velocities = None

    # robot enable and initialize
    def enable_robot(self):
        pass

    def enable_arm(self):
        pass

    def init_arm(self):
        pass

    def init_ros_node(self):
        pass

    def init_gripper(self):
        pass

    def init_robot(self):
        self.init_ros_node()
        self.enable_robot()
        self.enable_arm()
        self.init_arm()
        self.init_gripper()
        self.open_robot_gripper()

    # robot param set and get
    def set_init_position(self, init_position):
        self.__init_position = init_position

    def set_init_joint_angles(self, init_joint_angles):
        self.__init_joint_angles = init_joint_angles

    def set_grasp_center(self, grasp_center):
        self.__grasp_center = grasp_center

    def set_grasp_orient(self, grasp_orient):
        self.__grasp_orient = grasp_orient

    def set_grasp_offset(self, grasp_offset):
        self.__grasp_offset = grasp_offset

    def get_init_position(self):
        return self.__init_position

    def get_init_joint_angles(self):
        return self.__init_joint_angles

    def get_grasp_center(self):
        return self.__grasp_center

    def get_grasp_orient(self):
        return self.__grasp_orient

    def get_grasp_offset(self):
        return self.__grasp_offset

    # set and record robot state
    def set_gripper_velocity(self, speed):
        pass

    def get_gripper_velocity(self):
        pass

    def get_joint_angles(self):
        pass

    def get_end_effector_position(self):
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

    def move_robot_to_joint_angles(self, angles):
        pass

    def reset(self):
        pass

    def run_grasp(self):
        pass

