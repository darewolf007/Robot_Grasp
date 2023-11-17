import rospy
import logging
from intera_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest,)
from std_msgs.msg import Header
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion,)
from Robot_Grasp.srv import VmrnDetection, VmrnDetectionResponse

class ros1_function(object):
    def __init__(self, rate = 10):
        self._start_time = rospy.get_time()
        self.rate = rate
    def ros1_init_ros_node(self, node_name, anonymous=True):
        if not rospy.core.is_initialized():
            rospy.init_node(node_name, anonymous=anonymous)

    def ros1_robot_sleep(self, time):
        rospy.sleep(time)

    def ros1_log_config(self, log_path, log_name):
        rospy.set_param('/rosout/logging_level', 'DEBUG')
        rospy.set_param('/rosout/format', "[$time][$severity][$logger]: $msg")
        rospy.set_param('/rosout/output_file', log_path+log_name)

    def ros1_log(self, log_info, mode="debug"):
        if mode == "debug":
            rospy.logdebug(log_info)
        elif mode == "info":
            rospy.loginfo(log_info)
        elif mode == "warn":
            rospy.logwarn(log_info)
        elif mode == "error":
            rospy.logerr(log_info)
        elif mode == "fatal":
            rospy.logfatal(log_info)
        else:
            print("log mode unknown, please check")

    def ros1_done(self):
        if rospy.is_shutdown():
            return True

    def ros1_time_stamp(self):
        return rospy.get_time() - self._start_time

    def ros1_rate(self):
        return rospy.Rate(self.rate)

class Sawyer_connect_ros1(ros1_function):
    def __init__(self, arm_name):
        super(ros1_function, self).__init__()
        self.arm_name = arm_name
        self.inverse_kinematics_srv_name = "ExternalTools/" + self.arm_name + "/PositionKinematicsNode/IKService"

    def ros1_inverse_kinematics(self, position, orient):
        ros_position = Point(position[0], position[1], position[2])
        ros_orient = Quaternion(orient[0], orient[1], orient[2], orient[3])
        iksvc = rospy.ServiceProxy(self.inverse_kinematics_srv_name, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        header = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = {str(self.arm_name): PoseStamped(header=header, pose=Pose(position=ros_position, orientation=ros_orient))}
        ikreq.pose_stamp.append(poses[self.arm_name])
        ikreq.tip_names.append(self.arm_name + '_hand')
        try:
            rospy.wait_for_service(self.inverse_kinematics_srv_name, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1
        if (resp.result_type[0] > 0):
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            return limb_joints
        else:
            return -1

    def ros1_receive_grasp_infor(self):
        obj_grasp = rospy.ServiceProxy('vmrn_detection', VmrnDetection)
        response_grasp = obj_grasp.call(True)
        return response_grasp.grasp_center, response_grasp.grasp_ori

