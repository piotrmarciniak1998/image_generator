#!/usr/bin/env python3

import rospy
import os
import pickle
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel
from category import Category


def gazebo_spawn_model_client(model_name, model_xml, robot_namespace, initial_pose=Pose(), reference_frame="world"):
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    try:
        spawner = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        spawner(model_name=model_name,
                model_xml=model_xml,
                robot_namespace=robot_namespace,
                initial_pose=initial_pose,
                reference_frame=reference_frame)
    except rospy.ServiceException as e:
        print("Service call failed: ", e)


if __name__ == "__main__":
    abs_path = os.path.abspath("./src/image_generator/")

    # loading a pickle created by convert_obj_to_urdf.py script
    try:
        objects = pickle.load(open(abs_path + "/resources/pickle/categories.pickle", "rb"))
    except FileNotFoundError:
        print("FileNotFoundError: You need to execute convert_obj_to_urdf.py before executing this script!")
        quit()

    # spawning all of the tables in ../resources/urdf_files
    for category_number, category in enumerate(objects):
        for number in range(objects[category].len):
            gazebo_spawn_model_client(model_name=objects[category].names[number],
                                      model_xml=open(objects[category].urdf_files[number], 'r').read(),
                                      robot_namespace=category,
                                      initial_pose=Pose(position=Point(category_number, number, 0.1),
                                                        orientation=Quaternion(0.7071, 0, 0, 0.7071)))
