#!/usr/bin/env python3

import rospy
import os
import pickle
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel
from scripts.sources.item import Item
from random import randint


if __name__ == "__main__":
    abs_path = os.path.abspath("./src/image_generator/")

    # loading a pickle created by convert_obj_to_urdf.py script
    try:
        items = pickle.load(open(abs_path + "/resources/pickle/items.pickle", "rb"))
    except FileNotFoundError:
        print("FileNotFoundError: You need to execute convert_obj_to_urdf.py before executing this script!")
        quit()

    # All items are categorized

    # for category_number, category in enumerate(items):
    #     for item_number, item in enumerate(items[category]):
    #         item.normalize_position(pose=(category_number * 10, item_number * 10, 0),
    #                                 angle=randint(0, 360))
    #         item.spawn()
    #
    # input()
    # for category in items:
    #     for item in items[category]:
    #         item.despawn()

    # All items are put on top of each other

    height = 0
    for category in items:
        for item in items[category]:
            item.normalize_position(pose=(0, 0, height),
                                    angle=180)
            item.spawn()
            height += item.z_span

    input()
    for category in items:
        for item in items[category]:
            item.despawn()

