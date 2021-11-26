#!/usr/bin/env python3

import rospy
import os
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel


def gazebo_spawn_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame="world"):
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
    print(abs_path)

    table_urdfs = []
    table_names = []
    cup_urdfs = []
    cup_names = []

    for root, dirs, files in os.walk(abs_path + "/resources/urdf_files"):
        for file in files:
            if file.startswith("table"):
                table_urdfs.append(os.path.join(root, file))
                table_names.append(os.path.join(root, file).replace(".urdf", "").replace(abs_path + "/resources/urdf_files/", ""))

    print(table_urdfs)
    print(table_names)

    for i in range(len(table_names)):
        gazebo_spawn_model_client(model_name=table_names[i],
                                  model_xml=open(table_urdfs[i], 'r').read(),
                                  robot_namespace="/tables",
                                  initial_pose=Pose(position=Point(i, i, 1),
                                                    orientation=Quaternion(0.7071, 0, 0, 0.7071)),
                                  reference_frame="world")
