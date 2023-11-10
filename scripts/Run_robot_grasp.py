from Detect_Grasp.Sawyer_VMRN_Grasp import VMRN_Grasp
from Robot_Grasp.srv import VmrnDetection, VmrnDetectionResponse

def vmrn_grasp_callback(request):
    reponse = VmrnDetectionResponse()
    grasp_center, grasp_ori = vmrn_grasp.run(request)
    reponse.grasp_center = grasp_center
    reponse.grasp_ori = grasp_ori
    return reponse

if __name__ == "__main__":
    vmrn_grasp = VMRN_Grasp()
    rospy.init_node('vmrn_grasp_detect')
    rospy.Service('vmrn_detection', VmrnDetection, vmrn_grasp_callback)
    rospy.spin()