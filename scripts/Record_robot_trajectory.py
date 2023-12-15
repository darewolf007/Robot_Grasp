#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from Grasp_Anything.Robot_Control.Sawyer_control import Sawyer_control
import os
import rospy

if __name__ == "__main__":
    rospy.init_node('sawyer_trajectory_record')
    script_dir = os.path.dirname(os.path.abspath(__file__))
    sawyer_robot = Sawyer_control(script_dir + "/Grasp_Anything/Configs/Sawyer_control.yaml")
    sawyer_robot.init_robot()
    use_keyboard = rospy.get_param('~use_keyboard')
    try:
        while not rospy.is_shutdown():
            if use_keyboard:
                sawyer_robot.thread_record_trajectory()
            else:
                sawyer_robot.record_trajectory()
    except Exception as e:
        print("something bad happend when record_trajectory!!!!")
        print("error is: ", e)
