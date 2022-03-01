#!/usr/bin/env python3

import rospy
import os
import pickle
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel
from category import Category
from random import randrange, sample, random


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


def random_Point(z, range_stop):
    while True:
        x = (random() - 0.5) * range_stop * 2
        y = (random() - 0.5) * range_stop * 2
        if x * x + y * y < range_stop + range_stop:
            return Point(x, y, z)


if __name__ == "__main__":
    abs_path = os.path.abspath("./src/image_generator/")

    # loading a pickle created by convert_obj_to_urdf.py script
    try:
        objects = pickle.load(open(abs_path + "/resources/pickle/categories.pickle", "rb"))
    except FileNotFoundError:
        print("FileNotFoundError: You need to execute convert_obj_to_urdf.py before executing this script!")
        quit()

    # spawning random table from urdf files
    random_table_number = randrange(objects["table"].len)
    random_table_number = 2  #temp

    gazebo_spawn_model_client(model_name=objects["table"].names[random_table_number],
                              model_xml=open(objects["table"].urdf_files[random_table_number], 'r').read(),
                              robot_namespace="table",
                              initial_pose=Pose(position=Point(0, 0, objects["table"].scale * 0.3),
                                                orientation=Quaternion(0.7071, 0, 0, 0.7071)))

    # TODO: make table glued to the world

    category = "something"
    random_models_number = 5
    random_models_list = sample(range(objects[category].len), random_models_number)
    print(random_models_list)

    for number in random_models_list:
        gazebo_spawn_model_client(model_name=objects[category].names[number],
                                  model_xml=open(objects[category].urdf_files[number], 'r').read(),
                                  robot_namespace=category,
                                  initial_pose=Pose(position=random_Point(z=objects["table"].scale * 0.5,
                                                                          range_stop=objects["table"].scale),
                                                    orientation=Quaternion(0.7071, 0, 0, 0.7071)))
