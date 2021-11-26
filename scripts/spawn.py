#!/usr/bin/env python3
from gazebo_msgs.srv import SpawnModel
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion


rospy.wait_for_service("/gazebo/spawn_urdf_model")
try:
    spawner = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
    spawner("table", open('/home/szymon/catkin_TS/src/image_generator/data/table.urdf', 'r').read(),
            "/tables", Pose(position=Point(1, 1, 1), orientation=Quaternion(0.7071, 0, 0, 0.7071)), "world")
except rospy.ServiceException as e:
    print("Service call failed: ",e)
