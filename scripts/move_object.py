#!/usr/bin/env python3

import rospy
import os
import pickle
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from category import Category


def gazebo_get_model_pose_client(model_name, relative_entity_name="world"):
    rospy.wait_for_service("/gazebo/get_model_state")
    try:
        get_model_state = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
        model_state = get_model_state(model_name=model_name,
                                      relative_entity_name=relative_entity_name)
        return model_state.pose
    except rospy.ServiceException as e:
        print("Service call failed: ", e)


def gazebo_set_model_pose_client(model_name, pose, twist=Twist(), reference_frame="world"):
    rospy.wait_for_service("/gazebo/set_model_state")
    try:
        set_model_state = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
        set_model_state(model_state=ModelState(model_name=model_name,
                                               pose=pose,
                                               twist=twist,
                                               reference_frame=reference_frame))
    except rospy.ServiceException as e:
        print("Service call failed: ", e)


def change_position(current_pose, vector):
    old_position_x = current_pose.position.x
    old_position_y = current_pose.position.y
    old_position_z = current_pose.position.z
    new_position = Point(old_position_x + vector[0],
                         old_position_y + vector[1],
                         old_position_z + vector[2])
    orientation = current_pose.orientation
    return Pose(new_position, orientation)


if __name__ == "__main__":
    abs_path = os.path.abspath("./src/image_generator/")

    # loading a pickle created by convert_obj_to_urdf.py script
    try:
        object_categories = pickle.load(open(abs_path + "/scripts/categories.pickle", "rb"))
    except FileNotFoundError:
        print("FileNotFoundError: You need to execute convert_obj_to_urdf.py before executing this script!")
        quit()

    # this will work only if spawn.py was executed first
    pose = gazebo_get_model_pose_client(model_name=object_categories["table"].names[0])
    pose = change_position(pose, (0, 0, 3))  # teleporting table0 to 3 blocks higher position
    print(pose)
    gazebo_set_model_pose_client(model_name=object_categories["table"].names[0],
                                 pose=pose)
