#!/usr/bin/env python3

import rospy
import os
import pickle
from camera import Camera

if __name__ == "__main__":
    abs_path = os.path.abspath("./src/image_generator/")

    # loading a pickle created by convert_obj_to_urdf.py script
    try:
        items = pickle.load(open(abs_path + "/resources/pickle/items.pickle", "rb"))
    except FileNotFoundError:
        print("FileNotFoundError: You need to execute convert_obj_to_urdf.py before executing this script!")
        quit()

    kinect = Camera("kinect", "/kinect/color/image_raw", "/kinect/depth/image_rect_raw")
    rospy.init_node("kinect_subscriber", anonymous=True)

    mug = items["mug"][2]
    mug.normalize_position()
    mug.spawn()

    pos = mug.get_pose()
    for i in range(0, 360, 10):
        kinect.move(pos, 2, i)
        kinect.take_photo("rgb")
        kinect.take_photo("depth")

    mug.despawn()

    # for i in range(0, 360, 10):
    #     input()
    #     kinect.move((0, 0, 0.5), i)
