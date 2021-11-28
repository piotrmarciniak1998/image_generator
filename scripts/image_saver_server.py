#!/usr/bin/env python3

import cv2
import os
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from image_generator.srv import ImageToSave, ImageToSaveResponse


def handle_image_to_save(req):
    bridge = CvBridge()
    try:
        cv2_img = bridge.imgmsg_to_cv2(req.image, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        cv2.imwrite(os.path.abspath(f"./src/image_generator/images/{req.index}_{req.filename}.png"), cv2_img)
        return ImageToSaveResponse(True)
    return ImageToSaveResponse(False)


def image_saver_server():
    rospy.init_node("image_saver_server")
    os.makedirs(os.path.abspath("./src/image_generator/images"), exist_ok=True)
    s = rospy.Service("/image_generator/image_saver", ImageToSave, handle_image_to_save)
    rospy.spin()


if __name__ == "__main__":
    try:
        image_saver_server()
    except rospy.ROSInterruptException:
        pass
