#!/usr/bin/env python3

import rospy
import os
import pickle
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel
from random import randint, randrange, uniform
from item import Item
from camera import Camera
from utils import position_on_item, get_average_x_y


# TODO: compare performance of the implemented method to spawning items once and only moving them on the scene

TABLE_CATEGORY = "table"  # label of table category
TARGET_CATEGORY = "mug"  # label of target category
TARGET_NUMBER = 0  # index of chosen item in target category
RANDOM_ITEMS = 10  # how many unrelated items to spawn on the table
MIN_CAMERA_DISTANCE = 1.5  # define the closest distance of the photo
MAX_CAMERA_DISTANCE = 3.0  # define the furthest distance of the photo
NUMBER_OF_ITERATIONS = 100  # define number of scenes to generate


if __name__ == "__main__":
    abs_path = os.path.abspath("./src/image_generator/")

    # loading a pickle created by convert_obj_to_urdf.py script
    try:
        items = pickle.load(open(abs_path + "/resources/pickle/items.pickle", "rb"))
    except FileNotFoundError:
        print("FileNotFoundError: You need to execute convert_obj_to_urdf.py before executing this script!")
        quit()

    # creating camera object
    kinect = Camera(name="kinect",
                    topic_rgb="/kinect/color/image_raw",
                    topic_depth="/kinect/depth/image_rect_raw")
    rospy.init_node("kinect_subscriber", anonymous=True)

    for iteration in range(NUMBER_OF_ITERATIONS):
        # spawning random table from urdf files
        table_number = randrange(len(items[TABLE_CATEGORY]))
        table = items[TABLE_CATEGORY][table_number]
        table.normalize_position()
        table.spawn()

        # spawning the obstructed mug (target)
        target = items[TARGET_CATEGORY][TARGET_NUMBER]
        target.normalize_position(pose=position_on_item(table),
                                  angle=randint(0, 360))
        target.spawn()

        # moving camera to different angles, pointed towards the mug
        kinect.move(pose=target.get_pose(),
                    distance=uniform(MIN_CAMERA_DISTANCE, MAX_CAMERA_DISTANCE),
                    angle=randint(0, 360))

        # defining categories of random objects on the scene (excluding table and target category)
        categories = list(items.keys())
        categories.remove(TABLE_CATEGORY)
        categories.remove(TARGET_CATEGORY)

        # spawning obstructing item
        obstructor_category = categories[randrange(len(categories))]
        obstructor_number = randrange(len(items[obstructor_category]))
        obstructor = items[obstructor_category][obstructor_number]
        # TODO:
        #  1. check if spawning the obstructor in the way of camera works
        #  2. add randomization in the future
        obstructor_position = (get_average_x_y(target.get_pose(), kinect.get_pose()), table.z_span)
        obstructor.normalize_position(pose=obstructor_position,
                                      angle=randint(0, 360))
        obstructor.spawn()

        # spawning random items on the scene
        random_items = []
        for i in range(RANDOM_ITEMS):
            random_category = categories[randrange(len(categories))]
            while True:
                random_number = randrange(len(items[random_category]))
                if not items[random_category][random_number].spawned:
                    break

            random_item = items[random_category][random_number]
            random_items.append(random_item)
            random_item.normalize_position(pose=position_on_item(table),
                                           angle=randint(0, 360))
            random_item.spawn()

        # taking photo of obstructed view on target
        kinect.take_photo(kind="rgb",
                          additional_text="_ob")
        kinect.take_photo(kind="depth",
                          additional_text="_ob")

        obstructor.despawn()

        # taking photo of unobstructed view on target
        kinect.take_photo(kind="rgb",
                          additional_text="_un")
        kinect.take_photo(kind="depth",
                          additional_text="_un")

        kinect.index += 1

        # despawning all the items on the scene
        for random_item in random_items:
            random_item.despawn()
        target.despawn()
        table.despawn()
