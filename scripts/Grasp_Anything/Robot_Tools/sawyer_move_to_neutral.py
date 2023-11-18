import rospy
import intera_interface
rospy.init_node("move_neutral", anonymous=True)
robot = intera_interface.RobotEnable()
limb_handle = intera_interface.Limb("right")
limb_handle.move_to_neutral()