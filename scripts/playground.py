#!/usr/bin/env python3

import rospy
import os
import pickle
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from gazebo_msgs.srv import GetModelState, SetModelState, SpawnModel, DeleteModel, GetModelProperties
from gazebo_msgs.msg import ModelState
from category import Category


objects = None
spawned_models = []  # spawned_models should be loaded dynamically in the future


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


def gazebo_delete_model_client(model_name):
    rospy.wait_for_service("/gazebo/delete_model")
    try:
        deleter = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        deleter(model_name=model_name)
    except rospy.ServiceException as e:
        print("Service call failed: ", e)


def gazebo_get_model_properties_client(model_name):
    rospy.wait_for_service("/gazebo/get_model_properties")
    try:
        get_model_properties = rospy.ServiceProxy("gazebo/get_model_properties", GetModelProperties)
        model_properties = get_model_properties(model_name=model_name)
        return model_properties
    except rospy.ServiceException as e:
        print("Service call failed: ", e)


def gazebo_get_model_state_client(model_name, relative_entity_name="world"):
    rospy.wait_for_service("/gazebo/get_model_state")
    try:
        get_model_state = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
        model_state = get_model_state(model_name=model_name,
                                      relative_entity_name=relative_entity_name)
        return model_state
    except rospy.ServiceException as e:
        print("Service call failed: ", e)


def gazebo_set_model_state_client(model_name, pose, twist=Twist(), reference_frame="world"):
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


def does_model_exist(query):
    global objects
    for category in objects:
        if query in objects[category].names:
            return category, objects[category].names.index(query)
    return None, None


def spawn_function():
    global spawned_models
    global objects
    print("\nModels:")
    for category in objects:
        print(" " + category)
        for model_number in range(objects[category].len):
            print("  " + objects[category].names[model_number])

    while True:
        decision = input("Model: ")
        category, index = does_model_exist(decision)
        if category:
            break
        else:
            print("Choose only available models!")

    gazebo_spawn_model_client(model_name=decision,
                              model_xml=open(objects[category].urdf_files[index], 'r').read(),
                              robot_namespace=category)
    spawned_models.append(decision)


def delete_function():
    global spawned_models
    print("\nModels:")
    for model in spawned_models:
        print(" " + model)

    while True:
        decision = input("Model: ")
        if decision in spawned_models:
            break
        else:
            print("Choose only available models!")

    gazebo_delete_model_client(model_name=decision)
    spawned_models.remove(decision)


def get_state_function():
    global spawned_models
    print("\nModels:")
    for model in spawned_models:
        print(" " + model)

    while True:
        decision = input("Model: ")
        if decision in spawned_models:
            break
        else:
            print("Choose only available models!")

    model_state = gazebo_get_model_state_client(model_name=decision)
    print(model_state)


def get_properties_function():
    global spawned_models
    print("\nModels:")
    for model in spawned_models:
        print(" " + model)

    while True:
        decision = input("Model: ")
        if decision in spawned_models:
            break
        else:
            print("Choose only available models!")

    model_properties = gazebo_get_model_properties_client(model_name=decision)
    print(model_properties)


def move_function():
    global spawned_models
    print("\nModels:")
    for model in spawned_models:
        print(" " + model)

    while True:
        decision = input("Model: ")
        if decision in spawned_models:
            break
        else:
            print("Choose only available models!")

    pose = gazebo_get_model_state_client(model_name=decision).pose
    print("Move by vector [x, y, z]:")
    x = float(input("x = "))
    y = float(input("y = "))
    z = float(input("z = "))
    pose = change_position(pose, (x, y, z))
    gazebo_set_model_state_client(model_name=decision,
                                  pose=pose)


if __name__ == "__main__":
    abs_path = os.path.abspath("./src/image_generator/")

    # loading a pickle created by convert_obj_to_urdf.py script
    try:
        objects = pickle.load(open(abs_path + "/resources/pickle/categories.pickle", "rb"))
    except FileNotFoundError:
        print("FileNotFoundError: You need to execute convert_obj_to_urdf.py before executing this script!")
        quit()

    # UI could see improvements in the future
    while True:
        print("\nActions: ")
        print(" s - spawn model")
        print(" d - delete model")
        print(" g_s - get state of model")
        print(" g_p - get properties of model")
        print(" m - move model")
        print(" q - quit")
        decision = input("Action: ")
        if decision == "s":
            spawn_function()
        elif decision == "d":
            delete_function()
        elif decision == "g_s":
            get_state_function()
        elif decision == "g_p":
            get_properties_function()
        elif decision == "m":
            move_function()
        elif decision == "q":
            quit()
        else:
            print("Choose only available actions!")
