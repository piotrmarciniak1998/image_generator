import rospy
from math import radians, cos, sin
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, GetModelState


class Item:
    def __init__(self, name, category, obj_path, urdf_path, scale, x_min, x_max, y_min, y_max, z_min, z_max,
                 pose=Pose(), reference_frame="world"):
        self.name = name
        self.category = category
        self.obj_path = obj_path
        self.urdf_path = urdf_path
        self.pose = pose
        self.reference_frame = reference_frame

        self.scale = scale
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = z_min
        self.z_max = z_max

        self.x_span = x_max - x_min
        self.y_span = y_max - y_min
        self.z_span = z_max - z_min

        self.used = False

    def normalize_position(self, pose=(0, 0, 0), angle=0):
        if pose == (0, 0, -10):
            self.used = False
        else:
            self.used = True
        q = quaternion_from_euler(radians(90), 0, radians(angle))
        self.pose = Pose(position=Point(pose[0] - ((self.x_min + self.x_max) * cos(radians(angle)) / 2 +
                                                   (self.y_min + self.y_max) * sin(radians(angle)) / 2),
                                        pose[1] - ((self.x_min + self.x_max) * sin(radians(angle)) / 2 +
                                                   (self.y_min + self.y_max) * cos(radians(angle)) / 2),
                                        pose[2] - self.z_min),
                         orientation=Quaternion(q[0], q[1], q[2], q[3]))

    def get_bounding_box_of_normalized_item(self):
        x_min = -self.x_span / 2
        x_max = self.x_span / 2
        y_min = -self.y_span / 2
        y_max = self.y_span / 2
        z = self.z_span

        return x_min, x_max, y_min, y_max, z

    def get_pose(self):
        return (self.pose.position.x, self.pose.position.y, self.pose.position.z)

    def spawn(self):
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        try:
            spawner = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
            spawner(model_name=self.name,
                    model_xml=open(self.urdf_path, 'r').read(),
                    robot_namespace=self.category,
                    initial_pose=self.pose,
                    reference_frame=self.reference_frame)
        except rospy.ServiceException as e:
            print("Service call failed: ", e)

    def despawn(self):
        rospy.wait_for_service("/gazebo/delete_model")
        try:
            deleter = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
            deleter(model_name=self.name)
        except rospy.ServiceException as e:
            print("Service call failed: ", e)

    def move(self, pose=(0, 0, 0), angle=0):
        self.normalize_position(pose=pose, angle=angle)
        rospy.wait_for_service("/gazebo/set_model_state")
        try:
            set_model_state = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
            response = set_model_state(model_state=ModelState(model_name=self.name,
                                                              pose=self.pose,
                                                              twist=Twist(),
                                                              reference_frame=self.reference_frame))
        except rospy.ServiceException as e:
            print("Service call failed: ", e)
        else:
            return response
