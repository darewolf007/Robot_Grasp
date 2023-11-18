from Robot_Grasp.srv import VmrnDetection, VmrnDetectionResponse
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion,)
from Grasp_Anything.Robot_Control.Sawyer_control import Sawyer_control
import os
import rospy
import numpy as np

if __name__ == "__main__":
    rospy.init_node('vmrn_grasp_detect')
    script_dir = os.path.dirname(os.path.abspath(__file__))
    sawyer_robot = Sawyer_control(script_dir + "/Grasp_Anything/Configs/Sawyer_control.yaml")
    sawyer_robot.init_robot()
    sawyer_robot.set_putdown_position([0.8744617820936951, -0.10845370509646012, 0.359])
    sawyer_robot.run(np.array([0.,0,0.08]))
    rospy.spin()