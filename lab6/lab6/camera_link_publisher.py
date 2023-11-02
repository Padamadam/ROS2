import rclpy
from visualization_msgs.msg import Marker
#from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.node import Node
from math import cos, sin, atan2
import math
import random

class CameraLinkPublisher(Node):

    def __init__(self):
        super().__init__('camera_link_publisher')

        self.publisher_ = self.create_publisher(PoseArray, 'camera_link', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        if self.count < 100:
            if self.count % 10 == 1:
                self.publish()
            self.count += 1

    def publish(self):
        print('publish')
        rot = self.euler_to_quaternion(0.0, 0.0, random.uniform(-3.14, 3.14))
        msg = PoseArray()
        msg.poses = [Pose(), Pose()]
        msg.poses[0].position.x = random.uniform(-0.05, 0.05)
        msg.poses[0].position.y = random.uniform(-0.05, 0.05)
        msg.poses[0].position.z = 0.245
        msg.poses[0].orientation.x = rot.x
        msg.poses[0].orientation.y = rot.y
        msg.poses[0].orientation.z = rot.z
        msg.poses[0].orientation.w = rot.w
        
        rot = self.euler_to_quaternion(0.0, 0.0, random.uniform(-3.14, 3.14))

        msg.poses[1].position.x = random.uniform(-0.05, 0.05)
        msg.poses[1].position.y = random.uniform(-0.05, 0.05)
        msg.poses[1].position.z = 0.235
        msg.poses[1].orientation.x = rot.x
        msg.poses[1].orientation.y = rot.y
        msg.poses[1].orientation.z = rot.z
        msg.poses[1].orientation.w = rot.w

        msg.header.frame_id = "camera_link"
        self.publisher_.publish(msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    random.seed()
    rclpy.init()
    node = CameraLinkPublisher()
    rclpy.spin(node)

