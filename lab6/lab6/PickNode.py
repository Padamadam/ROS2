#from dobot_msgs.srv import GripperControl
#from dobot_msgs.action import PointToPoint

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker

from time import sleep
from std_msgs.msg import String

from sensor_msgs.msg import JointState
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from math import cos, sin, atan2, sqrt
from geometry_msgs.msg import Quaternion
from ros2_aruco_interfaces.msg import ArucoMarkers
from sensor_msgs.msg import JointState

class PickNode(Node):

    def __init__(self):
        super().__init__('PickNode')

        self.subscription = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.listener_callback,
            10)
        
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(
            JointState, 
            'dobot_joint_states', 
            10)

        self.cube_publisher = self.create_publisher(
            Marker,
            "/marker_cube",
            10
        )

        self.sheet_publisher = self.create_publisher(
            Marker,
            "/marker_sheet",
            10
        )


        self.pos_publisher = self.create_publisher(Quaternion, 'gripper_pose', 10)
        self.joints_new = (0.0, 0.0, 0.0, 0.0)
        self.joints = [0.0, 0.0, 0.0, 0.0]
        self.cube = (0.0, 0.0, 0.0, 0.0)
        self.paper = (0.0, 0.0, 0.0, 0.0)
        self.remaining = 0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = 0
        self.target = (0.0, 0.0, 0.0, 0.0)
        self.pos = [0.2, 0.0, 0.12, 0.0]

    def timer_callback(self):
        if self.state == 1 and self.remaining == 0:
            x, y, z, theta5 = self.cube
            self.target = (x, y, z+0.05, theta5)
            self.state = 2
            self.remaining = 20
        if self.state == 2 and self.remaining == 0:
            self.target = self.cube
            self.state = 3
            self.remaining = 10
        if self.state == 3 and self.remaining == 0:
            x, y, z, theta5 = self.cube
            self.target = (x, y, z+0.05, theta5)
            self.state = 4
            self.remaining = 10
        if self.state == 4 and self.remaining == 0:
            x, y, z, theta5 = self.paper
            self.target = (x, y, z+0.05, theta5)
            self.state = 5
            self.remaining = 20
        if self.state == 5 and self.remaining == 0:
            self.target = self.paper
            self.state = 6
            self.remaining = 10
        if self.state == 6 and self.remaining == 0:
            x, y, z, theta5 = self.paper
            self.target = (x, y, z+0.05, theta5)
            self.state = 7
            self.remaining = 10
        if self.state == 7 and self.remaining == 0:
            x, y, z, theta5 = self.paper
            self.target = (0.2, 0.0, 0.12, 0.0)
            self.state = 0
            self.remaining = 10
        print(self.target)
        print(self.pos)
        if self.state >= 4 and self.state <= 6:
            print('moving cube')
            gripper_pos = Quaternion(x=self.pos[0], y=self.pos[1], z=self.pos[2], w=self.pos[3])
            self.pos_publisher.publish(gripper_pos)

        if self.remaining > 0:
            for i in range(4):
                self.pos[i] += (self.target[i] - self.pos[i])/self.remaining
            self.remaining -= 1
        self.calculatePosition(self.pos)
        t1, t2, t3, t5 = self.joints
        pos = [t1, t2, t3-t2, -t3, -t5]
        msg = JointState()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.name = ["joint_bar1_to_base", "joint_bar2_to_bar1", "joint_bar3_to_bar2", "joint_bar4_to_bar3", "joint_gripper_to_bar4"]
        msg.position = pos
        msg.header.frame_id = "base"
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        if self.state == 0 or msg.id == 2:
            self.state = 1
            q = msg.pose.orientation
            angle = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            pos = msg.pose.position
            if(msg.id == 1):
                self.cube = (pos.x, pos.y, pos.z - 0.01, angle)
            elif(msg.id == 2):
                self.paper = (pos.x, pos.y, pos.z, angle)
        self.poses = msg.poses

        self.marker_ids = msg.marker_ids
        self.publish2()

    def calculatePosition(self, pos):
        x, y, z, theta5 = pos
        h = 0.138
        l1 = 0.135
        l2 = 0.147
        l3 = 0.05
        l4 = 0.125
        z -= h
        z += l4
        x = sqrt(x*x + y*y)
        x -= l3
        d = sqrt(x*x + z*z)
        gamma = math.pi
        alpha = 0.0
        try:
            gamma = math.acos((l1*l1+l2*l2-d*d)/(2*l1*l2))
            alpha = math.asin(l2*sin(gamma)/d)
        except ValueError:
            pass
        theta2 = math.atan2(x, z) - alpha
        theta3 = math.pi / 2 - gamma
        theta1 = math.atan2(pos[1], pos[0])
        self.joints = [theta1, theta2, (theta2 + theta3), (theta5 - theta1)]
        # print(self.joints)

    # def callback(self, msg):
    #     self.poses = msg.poses

    #     self.marker_ids = msg.marker_ids
    #     self.publish2()


    
    def publish2(self):
        # self.get_logger().info("callback entered")
        for i in range(len(self.poses)):
            # self.get_logger().info(f"{len(msg.poses)}\n\n\n\n\n\n\n\n\n")
            if self.marker_ids[i] == 13:
                # self.get_logger().info("cube entered")
                cube_marker = Marker()
                cube_marker.header.frame_id = 'camera_link'
                cube_marker.type = Marker.CUBE
                cube_marker.action = Marker.ADD
                cube_marker.id = 13

                cube_marker.pose.position.x = self.poses[i].position.y
                cube_marker.pose.position.y = -self.poses[i].position.x
                cube_marker.pose.position.z = self.poses[i].position.z + 0.006

                cube_marker.pose.orientation.x = self.poses[i].orientation.x
                cube_marker.pose.orientation.y = self.poses[i].orientation.y
                cube_marker.pose.orientation.z = 0.0
                cube_marker.pose.orientation.w = 0.0

                cube_marker.scale.x = 0.02
                cube_marker.scale.y = 0.02
                cube_marker.scale.z = 0.02

                cube_marker.color.a = 1.0
                cube_marker.color.r = 1.0
                cube_marker.color.g = 0.6
                cube_marker.color.b = 0.0
                self.cube_publisher.publish(cube_marker)

            
            if self.marker_ids[i] == 1:

                paper_sheet_marker = Marker()
                paper_sheet_marker.header.frame_id = "camera_link"
                paper_sheet_marker.type = Marker.CUBE
                paper_sheet_marker.action = Marker.ADD
                paper_sheet_marker.id = 1


                paper_sheet_marker.pose.position.x = self.poses[i].position.y
                paper_sheet_marker.pose.position.y = -self.poses[i].position.x
                paper_sheet_marker.pose.position.z = self.poses[i].position.z

                paper_sheet_marker.pose.orientation.x = self.poses[i].orientation.x
                paper_sheet_marker.pose.orientation.y = self.poses[i].orientation.y
                paper_sheet_marker.pose.orientation.z = 0.0
                paper_sheet_marker.pose.orientation.w = 0.0

                paper_sheet_marker.scale.x = 0.075
                paper_sheet_marker.scale.y = 0.105
                paper_sheet_marker.scale.z = 0.001

                paper_sheet_marker.color.a = 1.0
                paper_sheet_marker.color.r = 1.0
                paper_sheet_marker.color.g = 1.0
                paper_sheet_marker.color.b = 1.0
                self.sheet_publisher.publish(paper_sheet_marker)


def main():
    rclpy.init()
    node = PickNode()
    rclpy.spin(node)
