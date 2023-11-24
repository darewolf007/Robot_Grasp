from Grasp_Anything.Robot_Control.Sawyer_control import Sawyer_control
import os
import rospy
import keyboard

def trajectory_recorder(robot):
    robot.record_trajectory()

# def keyboard_callback(event):
#     if event.event_type == keyboard.KEY_DOWN:
#         if event.name == 'a':  # 例如，按下 'a' 键来触发服务
#             trajectory_recorder()
#         elif event.name == 'q':
#             rospy.signal_shutdown("Quit")
#         else:
#             pass

def keyboard_callback(event, robot, should_record):
    if event.event_type == keyboard.KEY_DOWN:
        if event.name == 'a':
            if should_record:
                trajectory_recorder(robot)
        elif event.name == 'q':
            should_record = False  # 停止记录轨迹
            rospy.loginfo("Stopped recording trajectory")

if __name__ == "__main__":
    rospy.init_node('sawyer_trajectory_record')
    script_dir = os.path.dirname(os.path.abspath(__file__))
    sawyer_robot = Sawyer_control(script_dir + "/Grasp_Anything/Configs/Sawyer_control.yaml")
    sawyer_robot.init_robot()
    should_record = True  # 设置一个标志来控制是否执行记录轨迹

    keyboard.hook(lambda event: keyboard_callback(event, sawyer_robot, should_record))
    rospy.loginfo("Press 'a' to record trajectory")
    rospy.loginfo("Press 'q' to stop recording")
    rospy.spin()  # 保持节点运行
