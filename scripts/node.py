#!/usr/bin/env python3
import time

import rospy
import numpy as np

import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
from sensors_demos_gazebo.srv import ImageToSave
# Instantiate CvBridge
bridge = CvBridge()

image = Image()
def image_callback(msg):
    global image
    image = msg

def image_sub():
    rospy.init_node("image_subscriber", anonymous=True)
    image_to_save = rospy.Subscriber("/kinect/color/image_raw", Image, image_callback)
    time.sleep(1)
    rospy.wait_for_service('image_saver')
    try:
        global image
        image_saver = rospy.ServiceProxy('image_saver', ImageToSave)
        resp = image_saver(image)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == '__main__':
    try:
        image_sub()
    except rospy.ROSInterruptException:
        pass
