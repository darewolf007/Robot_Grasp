import rospy
from intera_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest,)
from std_msgs.msg import Header
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion,)

class ros1_function(object):
    pass

class Sawyer_connect_ros1(ros1_function):
    def __init__(self, arm_name):
        self.arm_name = arm_name
        self.inverse_kinematics_srv_name = "ExternalTools/" + self.arm_name + "/PositionKinematicsNode/IKService"

    def ros1_init_ros_node(self, node_name):
        rospy.init_node(node_name, anonymous=True)

    def ros1_inverse_kinematics(self, position, orient):
        ros_position = Point(position[0], position[1], position[2])
        ros_orient = Quaternion(orient[0], orient[1], orient[2], orient[3])
        iksvc = rospy.ServiceProxy(self.inverse_kinematics_srv_name, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        header = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = {str(self.arm_name): PoseStamped(header=header, pose=Pose(position=ros_position, orientation=ros_orient))}
        ikreq.pose_stamp.append(poses[self.arm_name])
        try:
            rospy.wait_for_service(self.inverse_kinematics_srv_name, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1
        if (resp.isValid[0]):
            print("SUCCESS - Valid Joint Solution Found:")
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            return limb_joints
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
        return -1

    def ros1_robot_sleep(self, time):
        rospy.sleep(time)