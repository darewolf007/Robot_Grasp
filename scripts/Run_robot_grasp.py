from Grasp_Anything.Detect_Grasp.Sawyer_VMRN_Grasp import VMRN_Grasp
from Robot_Grasp.srv import VmrnDetection, VmrnDetectionResponse
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion,)
import os
import rospy

def vmrn_grasp_callback(request):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    vmrn_grasp = VMRN_Grasp(script_dir + "/Grasp_Anything/Configs/Sawyer_VMRN_Grasp.yaml")
    reponse = VmrnDetectionResponse()
    grasp_center, grasp_ori = vmrn_grasp.run(request)
    reponse.grasp_center = Point(grasp_center[0], grasp_center[1], grasp_center[2])
    reponse.grasp_ori = Quaternion(grasp_ori[0], grasp_ori[1], grasp_ori[2], grasp_ori[3])
    return reponse

if __name__ == "__main__":
    rospy.init_node('grasp_detect')
    rospy.Service('vmrn_detection', VmrnDetection, vmrn_grasp_callback)
    rospy.spin()