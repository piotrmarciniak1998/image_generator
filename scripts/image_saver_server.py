#!/usr/bin/env python3

import cv2
import os
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from image_generator.srv import ImageToSave, ImageToSaveResponse
import numpy as np

def handle_image_to_save(req):
    bridge = CvBridge()
    if 'depth' in req.filename:
        try:
            req.image.encoding = "32FC1"
            cv2_img = bridge.imgmsg_to_cv2(req.image, desired_encoding='passthrough')
            cv2_img = cv2_img * 63
        except CvBridgeError as e:
            print(e)
            return ImageToSaveResponse(False)
    elif 'rgb' in req.filename:
        try:
            req.image.encoding = "bgr8"
            cv2_img = bridge.imgmsg_to_cv2(req.image, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)
            return ImageToSaveResponse(False)
    os.makedirs(os.path.abspath("./src/image_generator/images"), exist_ok=True)
    cv2.imwrite(os.path.abspath(f"./src/image_generator/images/{req.index}_{req.filename}.png"), cv2_img)
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
#dataStencil