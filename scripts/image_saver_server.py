#!/usr/bin/env python3

import cv2
import os
import rospy
import numpy as np
import scipy.misc
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from image_generator.srv import ImageToSave, ImageToSaveResponse


def handle_image_to_save(req):
    bridge = CvBridge()
    if 'depth' in req.filename:
        req.image.encoding = "32FC1"
    elif 'rgb' in req.filename:
        req.image.encoding = "bgr8"

    try:
        cv2_img = bridge.imgmsg_to_cv2(req.image, desired_encoding='passthrough')
    except CvBridgeError as e:
        print(e)
        return ImageToSaveResponse(False)

    if 'depth' in req.filename:
        cv2_img = (cv2_img - 0.05) / 3.95
        cv2_img = cv2_img * 65535
        cv2_img = np.array(cv2_img, dtype=np.uint16)  # This line only change the type, not values


    elif 'rgb' in req.filename:
        cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)

    os.makedirs(os.path.abspath(f"{req.dirname}"), exist_ok=True)
    cv2.imwrite(os.path.abspath(f"{req.dirname}/{req.filename}.png"), cv2_img)
    return ImageToSaveResponse(True)


def image_saver_server():
    rospy.init_node("image_saver_server")
    s = rospy.Service("/image_generator/image_saver", ImageToSave, handle_image_to_save)
    rospy.spin()


if __name__ == "__main__":
    try:
        image_saver_server()
    except rospy.ROSInterruptException:
        pass
