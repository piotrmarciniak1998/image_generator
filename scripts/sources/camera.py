#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from math import radians, cos, sin, tan
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import Image
from image_generator.srv import ImageToSave
from cv_bridge import CvBridge


class Camera:
    def __init__(self, name, topic_rgb, topic_depth, pose=Pose(), reference_frame="world", distance=2):
        self.name = name
        self.pose = pose
        self.reference_frame = reference_frame

        self.distance = distance
        self.image = {"rgb": Image(),
                      "depth": Image()}
        self.is_ready = {"rgb": False,
                         "depth": False}

        rospy.Subscriber(topic_rgb, Image, self.image_rgb_callback)
        rospy.Subscriber(topic_depth, Image, self.image_depth_callback)

    def get_pose(self):
        return (self.pose.position.x, self.pose.position.y, self.pose.position.z)

    def image_rgb_callback(self, msg):
        self.image["rgb"] = msg
        self.is_ready["rgb"] = True

    def image_depth_callback(self, msg):
        self.image["depth"] = msg
        self.is_ready["depth"] = True

    def move(self, pose=Pose(), distance=2, angle_y=0, angle_z=0):
        q = quaternion_from_euler(0, radians(angle_y), radians(angle_z + 180))
        self.distance = distance
        self.pose = Pose(position=Point(pose[0] + self.distance * cos(radians(angle_z)),
                                        pose[1] + self.distance * sin(radians(angle_z)),
                                        pose[2] + self.distance * tan(radians(angle_y))),
                         orientation=Quaternion(q[0], q[1], q[2], q[3]))
        rospy.wait_for_service("/gazebo/set_model_state")
        try:
            set_model_state = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
            set_model_state(model_state=ModelState(model_name=self.name,
                                                   pose=self.pose,
                                                   twist=Twist(),
                                                   reference_frame=self.reference_frame))
        except rospy.ServiceException as e:
            print("Service call failed: ", e)
        else:
            self.is_ready["rgb"] = False
            self.is_ready["depth"] = False

    def take_photo(self, dirname="", filename="depth", save=True):
        if "depth" in filename:
            kind = "depth"
        elif "rgb" in filename:
            kind = "rgb"
        else:
            return None

        if save:
            rospy.wait_for_service("/image_generator/image_saver")
            while not self.is_ready[kind]:
                pass

            try:
                image_saver = rospy.ServiceProxy("/image_generator/image_saver", ImageToSave)
                image_saver(dirname=dirname,
                            filename=filename,
                            image=self.image[kind])
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
            else:
                return self.image[kind]
        else:
            return self.image[kind]
