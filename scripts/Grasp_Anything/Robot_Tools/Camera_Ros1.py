import rospy
try:
    import Queue
except:
    import queue as Queue
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image, CameraInfo
import ros_numpy
import numpy as np
import time


class KinectDK(object):
    def __init__(self):
        self.queue_hbody = Queue.Queue(3)
        self.queue_color = Queue.Queue(3)
        self.queue_depth = Queue.Queue(3)
        self.queue_left_hand = Queue.Queue(3)
        self.rgb_info = None
        self.depth_info = None
        self.hbody_sub = rospy.Subscriber("/body_tracking_data", MarkerArray, self.hbody_callback)
        self.left_hand_sub = rospy.Subscriber('/cameras/left_hand_camera/image', Image, self.left_hand_callback)
        self.color_sub = rospy.Subscriber('/rgb/image_raw', Image, self.color_callback)
        self.depth_sub = rospy.Subscriber('/depth_to_rgb/image_raw', Image, self.depth_callback)
        self.rgbinfo_sub = rospy.Subscriber('/rgb/camera_info', CameraInfo, self.rgbinfo_callback)
        self.depthinfo_sub = rospy.Subscriber('/depth/camera_info', CameraInfo, self.depthinfo_callback)
        self.wait_init()

    def wait_init(self, timeout=100.):
        t = 0.
        while self.rgb_info is None or self.depth_info is None:
            time.sleep(0.1)
            t += 0.1
            if t >= timeout:
                raise Queue.Empty

    def rgbinfo_callback(self, camera_info):
        self.rgb_info = camera_info
        self.rgbinfo_sub.unregister()

    def depthinfo_callback(self, camera_info):
        self.depth_info = camera_info
        self.depthinfo_sub.unregister()

    def hbody_callback(self, marker_msg):
        if self.queue_hbody.full():
            self.queue_hbody.get_nowait()
        self.queue_hbody.put(marker_msg.markers)

    def color_callback(self, image_msg):
        cv_image = ros_numpy.numpify(image_msg)
        if self.queue_color.full():
            self.queue_color.get()
        self.queue_color.put(np.asarray(cv_image))

    def left_hand_callback(self, image_msg):
        cv_image = ros_numpy.numpify(image_msg)
        if self.queue_left_hand.full():
            self.queue_left_hand.get()
        self.queue_left_hand.put(np.asarray(cv_image))

    def depth_callback(self, image_msg):
        cv_image = ros_numpy.numpify(image_msg)
        if self.queue_depth.full():
            self.queue_depth.get()
        self.queue_depth.put(np.asarray(cv_image))

    def release(self):
        self.hbody_sub.unregister()
        self.color_sub.unregister()
        self.depth_sub.unregister()
        self.left_hand_sub.unregister()