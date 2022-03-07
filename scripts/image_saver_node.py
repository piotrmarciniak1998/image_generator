#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from image_generator.srv import ImageToSave


image = Image()
is_ready = False


def image_callback(msg):
    global image, is_ready
    image = msg
    is_ready = True


def image_saver_client(filename, index, image):
    rospy.wait_for_service("/image_generator/image_saver")
    try:
        image_saver = rospy.ServiceProxy("/image_generator/image_saver", ImageToSave)
        response = image_saver(filename=filename,
                               index=index,
                               image=image)
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def image_sub():
    rospy.init_node("image_subscriber", anonymous=True)
    rospy.Subscriber("/kinect/color/image_raw", Image, image_callback)

    global image, is_ready
    while not is_ready:
        pass

    print(image_saver_client(filename="image_test_rgb",
                             index=1258,
                             image=image))


if __name__ == '__main__':
    try:
        image_sub()
    except rospy.ROSInterruptException:
        pass
