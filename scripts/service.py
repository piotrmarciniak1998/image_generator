#!/usr/bin/env python3

from __future__ import print_function

import rospy
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import os
# Instantiate CvBridge
bridge = CvBridge()
from sensors_demos_gazebo.srv import *

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg.image, "bgr8")
        print("Save image")
    except CvBridgeError as e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
        #os.path.abspath("../images/camera_image.jpeg")
        print(os.path.abspath("./src/image_generator/images/camera_image.jpeg"))
        cv2.imwrite(os.path.abspath("./src/image_generator/images/camera_image.jpeg"), cv2_img)
def image_saver():
    rospy.init_node('image_saver_service')
    s = rospy.Service('image_saver', ImageToSave, image_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        image_saver()
    except rospy.ROSInterruptException:
        pass

