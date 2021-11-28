#!/usr/bin/env python3

import cv2
import os
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from image_generator.srv import ImageToSave


def image_callback(msg):
    bridge = CvBridge()
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg.image, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        cv2.imwrite(os.path.abspath("./src/image_generator/images/camera_image.jpeg"), cv2_img)


def image_saver():
    rospy.init_node("image_saver")
    s = rospy.Service("/image_generator/image_saver", ImageToSave, image_callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        image_saver()
    except rospy.ROSInterruptException:
        pass

