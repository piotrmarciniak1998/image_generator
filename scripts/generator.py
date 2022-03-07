#!/usr/bin/env python3

import rospy
import os
import pickle
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel
from item import Item
from camera import Camera
from random import randint, randrange, uniform


TABLE_CATEGORY = "table"
TARGET_CATEGORY = "mug"
OBSTRUCTOR_CATEGORY = "mug"
RANDOM_ITEMS = 10
CAMERA_DISTANCE = 2
NUMBER_OF_ITERATIONS = 100


def position_on_item(item):
    x_min, x_max, y_min, y_max, height = item.get_bounding_box_of_normalized_item()
    a = x_max * 0.9
    b = y_max * 0.9
    while True:
        x = uniform(x_min, x_max)
        y = uniform(y_min, y_max)

        # TODO: check for collisions with other objects
        if (x * x) / (a * a) + (y * y) / (b * b) < 1:
            return x, y, height


if __name__ == "__main__":
    abs_path = os.path.abspath("./src/image_generator/")

    # loading a pickle created by convert_obj_to_urdf.py script
    try:
        items = pickle.load(open(abs_path + "/resources/pickle/items.pickle", "rb"))
    except FileNotFoundError:
        print("FileNotFoundError: You need to execute convert_obj_to_urdf.py before executing this script!")
        quit()

    # creating camera object
    kinect = Camera("kinect", "/kinect/color/image_raw", "/kinect/depth/image_rect_raw", 2)
    rospy.init_node("kinect_subscriber", anonymous=True)

    for iteration in range(NUMBER_OF_ITERATIONS):
        # spawning random table from urdf files
        table_number = randrange(len(items[TABLE_CATEGORY]))
        table = items[TABLE_CATEGORY][table_number]
        table.normalize_position()
        table.spawn()

        # spawning the obstructed mug (target)
        target_number = randrange(len(items[TARGET_CATEGORY]))
        target = items[TARGET_CATEGORY][target_number]
        target.normalize_position(pose=position_on_item(table), angle=randint(0, 360))
        target.spawn()

        # moving camera to different angles, pointed towards the mug
        kinect.move(pose=target.get_pose(), angle=randint(0, 360))

        while True:
            obstructor_number = randrange(len(items[OBSTRUCTOR_CATEGORY]))
            if OBSTRUCTOR_CATEGORY != TARGET_CATEGORY or obstructor_number != table_number:
                break
        obstructor = items[OBSTRUCTOR_CATEGORY][obstructor_number]
        # TODO: spawning the obstructor in the way of camera
        obstructor.normalize_position(pose=position_on_item(table), angle=randint(0, 360))
        obstructor.spawn()

        # spawning random items on the scene
        categories = list(items.keys())
        categories.remove(TABLE_CATEGORY)
        random_items = []
        for i in range(RANDOM_ITEMS):
            random_category = categories[randrange(len(categories))]
            while True:
                random_item_number = randrange(len(items[random_category]))
                if random_category != TARGET_CATEGORY:
                    if random_category != OBSTRUCTOR_CATEGORY or random_item_number != obstructor_number:
                        break
                elif random_category != OBSTRUCTOR_CATEGORY:
                    if random_item_number != target_number:
                        break
                elif random_item_number != target_number and random_item_number != obstructor_number:
                    break
            random_item = items[random_category][random_item_number]
            random_items.append(random_item)
            random_item.normalize_position(pose=position_on_item(table), angle=randint(0, 360))
            random_item.spawn()

        kinect.take_photo("rgb", "_ob")
        kinect.take_photo("depth", "_ob")

        obstructor.despawn()

        kinect.take_photo("rgb", "_un")
        kinect.take_photo("depth", "_un")

        kinect.index += 1

        # despawning all the items on the scene
        for random_item in random_items:
            random_item.despawn()
        target.despawn()
        table.despawn()
