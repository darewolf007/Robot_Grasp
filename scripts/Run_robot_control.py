#!/usr/bin/env python
from Robot_Grasp.srv import VmrnDetection, VmrnDetectionResponse
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from Grasp_Anything.Robot_Control.Sawyer_control import Sawyer_control
import os
import rospy
import numpy as np

def shutdown_function():
    rospy.loginfo("Shutting down sawyer control")

if __name__ == "__main__":
    rospy.init_node('vmrn_grasp_detect')
    script_dir = os.path.dirname(os.path.abspath(__file__))
    sawyer_robot = Sawyer_control(script_dir + "/Grasp_Anything/Configs/Sawyer_control.yaml")
    sawyer_robot.init_robot()
    rospy.on_shutdown(shutdown_function)
    try:
        while not rospy.is_shutdown():
            sawyer_robot.set_putdown_position([ 0.10633945, -0.82393165,  0.64651128])
            sawyer_robot.run(grasp_offset=[0.02,-0.02,0.08], effector_offset = [0.,0,0.1], add_effector_offset=False)
            sawyer_robot.move_robot_to_init_pose()  
            sawyer_robot.run(grasp_offset=[0.02,-0.02,0.08], effector_offset = [0.,0,0.1], add_effector_offset=False)
            sawyer_robot.move_robot_to_init_pose()
    except Exception as e:
        print("something bad happend when record_trajectory!!!!")
        print("error is: ", e)
