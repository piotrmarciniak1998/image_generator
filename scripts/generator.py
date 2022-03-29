#!/usr/bin/env python3

import rospy
import os
import pickle
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel
from random import randint, randrange, uniform
from time import perf_counter
from sources.item import Item
from sources.camera import Camera
from sources.utils import position_on_item, calculate_occlusion


TABLE_CATEGORY = "table"  # label of table category
TARGET_CATEGORY = "mug"  # label of target category
TARGET_NUMBER = 2  # index of chosen item in target category
RANDOM_ITEMS = 5  # how many unrelated items to spawn on the table
MIN_CAMERA_DISTANCE = 2.0  # define the closest distance of the photo
MAX_CAMERA_DISTANCE = 3.0  # define the furthest distance of the photo
NUMBER_OF_ITERATIONS = 100  # define number of scenes to generate


if __name__ == "__main__":
    print("1. Initializing.")

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

    print("2. Spawning items on the scene.")

    # spawn every object that can be used during simulation
    for category in items:
        print(f" - {category}")
        if category == TARGET_CATEGORY:
            item = items[TARGET_CATEGORY][TARGET_NUMBER]
            item.normalize_position(pose=(0, 0, -10))
            item.spawn()
        else:
            for item in items[category]:
                item.normalize_position(pose=(0, 0, -10))
                item.spawn()

    # defining categories of random objects on the scene (excluding table and target category)
    categories = list(items.keys())
    categories.remove(TABLE_CATEGORY)
    categories.remove(TARGET_CATEGORY)

    print("3. Starting program's main loop.")

    time_init = perf_counter()
    for iteration in range(NUMBER_OF_ITERATIONS):
        # metrics displayed in terminal
        average_time = (perf_counter() - time_init) / (iteration + 0.000001)
        estimated_time = int(round(average_time * (NUMBER_OF_ITERATIONS - iteration), 0))
        estimated_seconds = estimated_time % 60
        estimated_minutes = (estimated_time // 60) % 60
        estimated_hours = estimated_time // 3600
        completion_percentage = round(iteration / NUMBER_OF_ITERATIONS * 100, 1)
        if estimated_hours > 0:
            print(f"{completion_percentage}% - {estimated_hours}h, {estimated_minutes}m, {estimated_seconds}s left.")
        elif estimated_minutes > 0:
            print(f"{completion_percentage}% - {estimated_minutes}m, {estimated_seconds}s left.")
        else:
            print(f"{completion_percentage}% - {estimated_seconds}s left.")

        # spawning random table from urdf files
        table_number = randrange(len(items[TABLE_CATEGORY]))
        table = items[TABLE_CATEGORY][table_number]
        table.move()

        # spawning the obstructed mug (target)
        target = items[TARGET_CATEGORY][TARGET_NUMBER]
        target_pose = position_on_item(table, tolerance=0.4)
        target_angle = randint(0, 360)

        # moving camera to different angles, pointed towards the mug
        kinect.move(pose=target_pose,
                    distance=uniform(MIN_CAMERA_DISTANCE, MAX_CAMERA_DISTANCE),
                    angle=randint(0, 360))

        # spawning obstructing item
        obstructor_category = categories[randrange(len(categories))]
        obstructor_number = randrange(len(items[obstructor_category]))
        obstructor = items[obstructor_category][obstructor_number]
        obstructor_pose = position_on_item(item=table,
                                           target_pose=target_pose,
                                           camera_pose=kinect.get_pose(),
                                           is_obstructor=True)
        obstructor_angle = randint(0, 360)

        # spawning random items on the scene
        used_items = [target, obstructor]
        for i in range(RANDOM_ITEMS):
            random_category = categories[randrange(len(categories))]
            while True:
                random_number = randrange(len(items[random_category]))
                if not items[random_category][random_number].used:
                    break

            random_item = items[random_category][random_number]
            used_items.append(random_item)
            random_item.move(pose=position_on_item(item=table,
                                                   colliders=used_items,
                                                   target_pose=target_pose,
                                                   camera_pose=kinect.get_pose()),
                             angle=randint(0, 360))

        # calculating obstruction of target based on comparing different images
        rospy.sleep(0.2)
        msg_empty = kinect.take_photo(kind="depth", save=False)
        rospy.sleep(0.2)

        obstructor.move(pose=obstructor_pose,
                        angle=obstructor_angle)

        rospy.sleep(0.2)
        msg_obstructor = kinect.take_photo(kind="depth", save=False)
        rospy.sleep(0.2)

        obstructor.move(pose=(0, 0, -10))
        target.move(pose=target_pose,
                    angle=target_angle)

        rospy.sleep(0.2)
        msg_target = kinect.take_photo(kind="depth", save=False)
        rospy.sleep(0.2)

        obstructor.move(pose=obstructor_pose,
                        angle=obstructor_angle)

        rospy.sleep(0.2)
        msg_target_obstructor = kinect.take_photo(kind="depth", save=False)
        rospy.sleep(0.2)

        occlusion = calculate_occlusion(msg_empty, msg_obstructor, msg_target, msg_target_obstructor)

        # taking photo of obstructed view on target
        rospy.sleep(0.2)
        # rgb photos are not necessary for the project
        # kinect.take_photo(kind="rgb", save=True, additional_text=f"_o_{occlusion}")
        kinect.take_photo(kind="depth", save=True, additional_text=f"_o_{occlusion}")
        rospy.sleep(0.2)

        obstructor.move(pose=(0, 0, -10))

        # taking photo of unobstructed view on target
        rospy.sleep(0.2)
        # rgb photos are not necessary for the project
        # kinect.take_photo(kind="rgb", save=True, additional_text=f"_u_{occlusion}")
        kinect.take_photo(kind="depth", save=True, additional_text=f"_u_{occlusion}")
        rospy.sleep(0.2)

        kinect.index += 1

        # moving all the items from the scene
        table.move(pose=(0, 0, -10))
        for used_item in used_items:
            used_item.move(pose=(0, 0, -10))

    # despawning all the items on the scene
    print("4. Closing.")
    for category in items:
        if category == TARGET_CATEGORY:
            item = items[TARGET_CATEGORY][TARGET_NUMBER]
            item.despawn()
        else:
            for item in items[category]:
                item.despawn()
    print("5. Done.")
