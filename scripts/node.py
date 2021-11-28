#!/usr/bin/env python3

import time
import rospy
from sensor_msgs.msg import Image
from image_generator.srv import ImageToSave


image = Image()


def image_callback(msg):
    global image
    image = msg


def image_sub():
    rospy.init_node("image_subscriber", anonymous=True)
    image_to_save = rospy.Subscriber("/kinect/color/image_raw", Image, image_callback)
    time.sleep(1)
    rospy.wait_for_service("/image_generator/image_saver")
    global image
    try:
        image_saver = rospy.ServiceProxy("/image_generator/image_saver", ImageToSave)
        resp = image_saver(image)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == '__main__':
    try:
        image_sub()
    except rospy.ROSInterruptException:
        pass
