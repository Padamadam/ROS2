import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from sensor_msgs.msg import JointState
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import numpy as np
from math import cos, sin
from geometry_msgs.msg import Quaternion

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('ForwardKin')
        self.subscription = self.create_subscription(JointState,'/joint_states',self.listener_callback,10)
        self.subscription  # prevent unused variable warning


        self.publisher_ = self.create_publisher(PoseStamped, 'curent_position', 10)

    def listener_callback(self, msg: JointState):
        t1, t2, t3, t4, t5 = msg.position
        a = self.calculatePosition(t1, t2, t3, t4,t5, 0.138 ,0.135, 0.147, 0.05, 0.0565)
        self.timer_callback(a)

    def timer_callback(self, pos):
        x, y, z, q = pos
        msg = PoseStamped()
        pose = Pose()
        position = Point()
        position.x = x
        position.y = y
        position.z = z
        pose.orientation= q
        pose.position = position
        msg.pose = pose
        msg.header.frame_id = "base"
        self.publisher_.publish(msg)

    def calculatePosition(self, theta1, theta2, theta3, theta4, theta5, l1, l2, l3, l4, l5):
        x = (l2*sin(theta2)+l3*cos( theta2+theta3) +l4*cos(theta2+theta3+theta4) + l5*cos(theta2+theta3+theta4)) * cos(theta1)
        y = (l2*sin(theta2)+l3*cos( theta2+theta3) +l4*cos(theta2+theta3+theta4) + l5*cos(theta2+theta3+theta4)) * sin(theta1)
        z = (l2*cos(theta2)-l3*sin( theta2+theta3) -l4*sin(theta2+theta3+theta4) - l5*sin(theta2+theta3+theta4)) + l1
        rz = theta1
        ry = theta2 + theta3 + theta4
        qz_global = self.euler_to_quaternion(0, 0, theta1)
        qy = self.euler_to_quaternion(0, theta2 + theta3 + theta4, 0)
        qz_local = self.euler_to_quaternion(0, 0, -theta5)
        q_local = self.multiply_quaternions(qy, qz_local)
        return x,y,z, self.multiply_quaternions(qz_global, qy)
   
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
        
    def multiply_quaternions(self, q1, q2):
        w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        y = q1.w * q2.y - q1.x * q2.y + q1.y * q2.w + q1.z * q2.x
        z = q1.w * q2.z + q1.x * q2.z - q1.y * q2.x + q1.z * q2.w
        mag = x*x + y*y + z*z + w*w
        mag = math.sqrt(mag)
        w /= mag
        x /= mag
        y /= mag
        z /= mag
        return Quaternion(x=x, y=y, z=z, w=w)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalSubscriber()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
