#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from math import atan2, cos, pi, sin, sqrt, tan
from math import acos, asin
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import time
import numpy as np
from numpy import matrix
from math import atan, acos
from numpy.linalg import inv
from std_msgs.msg import Int32

class ReverseKinematicNode(Node):
    def __init__(self):
        super().__init__("ReverseKinematic")
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.marker_subscriber = self.create_subscription(MarkerArray, "camera_vizualization_marker", self.marker_callback, 10)
        self.array_publisher = self.create_publisher(MarkerArray, "visualization_marker_array", 10)
        self.gripper_sub = self.create_subscription(PoseStamped, "gripper_pose", self.gripper_callback, 10)
        self.dh = 0.135
        self.uh = 0.147
        self.actual_x = 0.147
        self.actual_z = 0.273
        self.actual_y = 0.0
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.theta3 = 0.0
        self.theta4 = 0.0
        self.theta5 = 0.0
        self.cube_x = 1.0
        self.cube_y = 1.0
        self.cube_z = 1.0      
        self.cube_or_x = 0.0
        self.cube_or_y = 0.0
        self.cube_or_z = 0.0
        self.cube_or_w = 1.0
        self.sheet_x = 1.0
        self.sheet_y = 1.0
        self.sheet_z = 1.0      
        self.sheet_or_x = 0.0
        self.sheet_or_y = 0.0
        self.sheet_or_z = 0.0
        self.sheet_or_w = 1.0
 
        self.prev_theta2 = 0.0
        self.prev_theta3 = 0.0
        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.publish)
        self.received = False
        self.step = 200
        self.step_counter = 0
        self.old_theta1 = 0.0
        self.old_theta2 = 0.0
        self.old_theta3 = 0.0
        self.old_theta4 = 0.0
        self.old_theta5 = 0.0
        self.callback_counter = 0
        self.state = 0
        self.isGoing = False
        self.changePos = False

    def marker_callback(self, marker_arr):
        self.item = "CUBE"
        self.get_logger().info("marker clbck")
        cube = marker_arr.markers[0]
        sheet = marker_arr.markers[1]
        self.get_logger().info(f"kąt:{str(self.old_theta1)}")
        self.get_logger().info(f"xxxxxxx:{str(cube.pose.position.x)}")
        new_pos_cube = self.transpose(cube.pose.position)
        new_pos_sheet = self.transpose(sheet.pose.position)
        self.cube_x = float(new_pos_cube[0])
        self.cube_y = float(new_pos_cube[1])
        self.cube_z = float(new_pos_cube[2])
        self.cube_or_x = cube.pose.orientation.x
        self.cube_or_y = cube.pose.orientation.y
        self.cube_or_z = cube.pose.orientation.z
        self.cube_or_w = cube.pose.orientation.w

        self.sheet_x = float(new_pos_sheet[0][0])
        self.sheet_y = float(new_pos_sheet[1][0])
        self.sheet_z = float(new_pos_sheet[2][0])
        self.sheet_or_x = sheet.pose.orientation.x
        self.sheet_or_y = sheet.pose.orientation.y
        self.sheet_or_z = sheet.pose.orientation.z
        self.sheet_or_w = sheet.pose.orientation.w
        self.get_logger().info(str(self.cube_x))
        self.get_logger().info(str(self.cube_y))
        self.get_logger().info(str(self.cube_z))
        self.get_logger().info(str(self.cube_x))
        self.get_logger().info(str(self.cube_y))
        self.get_logger().info(str(self.cube_z))
        self.publish_markers("base_link")
        self.goto_item()

    def goto_item(self):
        if self.item == "CUBE":
            x = self.cube_x
            y = self.cube_y
            z = self.cube_z + 0.01
            or_w = self.cube_or_w
        if self.item == "DOWN":
            x = self.cube_x
            y = self.cube_y
            z = self.cube_z - 0.013
            or_w = self.cube_or_w
        if self.item == "SHEET":
            x = self.sheet_x
            y = self.sheet_y
            z = self.sheet_z + 0.05
            or_w = self.sheet_or_w
        if self.item == "LEAVE":
            x = self.sheet_x
            y = self.sheet_y
            z = self.sheet_z
            or_w = self.sheet_or_w
        if self.item == "RETURN":
            x = self.sheet_x
            y = self.sheet_y
            z = self.sheet_z + 0.1
            or_w = self.sheet_or_w

        try:
            l1 = self.elbow_length
            # korekta długości ramienia l2, 0.0015 to magiczna liczba srodkujaca do punktu wg plaszczyznie XY
            l2 = self.forearm_length + self.hand_width - 0.0015


            c1 = sqrt(pow(x, 2) + pow(y, 2))

            beta = acos((pow(c1, 2) + pow(z, 2) - pow(l1, 2) - pow(l2, 2)) / (2*l1*l2))            
            alfa = atan(c1/z) - atan2((l1*sin(beta)), (l1 + l2*cos(beta))) 

            # orientacja ramienia
            if x < 0 and y < 0:
                shoulder_joint = - pi + atan(y/x)
            elif x < 0 and y > 0:
                shoulder_joint = pi + atan(y/x)
            else:
                shoulder_joint = atan(y/x)

            elbow_wheel_joint = alfa
            forearm_wheel_joint = beta

            # poziomowanie grippera
            hand_joint= - alfa - beta + pi

            device_joint =  shoulder_joint

            self.theta1 = shoulder_joint
            self.theta2 = elbow_wheel_joint
            self.theta3 = forearm_wheel_joint
            self.theta4 = hand_joint
            self.theta5 = device_joint

        except:
            self.get_logger().info("Out of range.")
            self.theta1 = atan(y/x) + pi
            self.theta2 = 0.0
            self.theta3 = 0.0
            self.theta4 = 0.0
            self.theta5 = 0.0


        self.step_theta1 = (self.theta1 - self.old_theta1)/self.step
        self.step_theta2 = (self.theta2 - self.old_theta2)/self.step
        self.step_theta3 = (self.theta3 - self.old_theta3)/self.step
        self.step_theta4 = (self.theta4 - self.old_theta4)/self.step
        self.step_theta5 = (self.theta5 - self.old_theta5)/self.step
        self.changePos = True

    def transpose(self, point):
        a1 = self.base_height
        a2 = self.elbow_length
        a3 = self.forearm_length
        a4 = 0 
        a5 = 0.05

        alpha_values = [np.deg2rad(-90), 0, 0 ,np.deg2rad(90), 0]

        DH_01 = np.array([[cos(self.theta_values[0]), -sin(self.theta_values[0])*cos(alpha_values[0]), sin(self.theta_values[0])*sin(alpha_values[0]), a1*cos(self.theta_values[0])],
                        [sin(self.theta_values[0]), cos(self.theta_values[0])*cos(alpha_values[0]), -cos(self.theta_values[0])*sin(alpha_values[0]), a1*sin(self.theta_values[0])],
                        [ 0, sin(alpha_values[0]), cos(alpha_values[0]), a1 ],
                        [ 0, 0, 0, 1 ]])
        DH_12 = np.array([[cos(self.theta_values[1]), -sin(self.theta_values[1])*cos(alpha_values[1]), sin(self.theta_values[1])*sin(alpha_values[1]), a2*cos(self.theta_values[1])],
                        [sin(self.theta_values[1]), cos(self.theta_values[1])*cos(alpha_values[1]), -cos(self.theta_values[1])*sin(alpha_values[1]), a2*sin(self.theta_values[1])],
                        [ 0, sin(alpha_values[1]), cos(alpha_values[1]), a2 ],
                        [ 0, 0, 0, 1 ]])
        DH_23 = np.array([[cos(self.theta_values[2]), -sin(self.theta_values[2])*cos(alpha_values[2]), sin(self.theta_values[2])*sin(alpha_values[2]), a3*cos(self.theta_values[2])],
                        [sin(self.theta_values[2]), cos(self.theta_values[2])*cos(alpha_values[2]), -cos(self.theta_values[2])*sin(alpha_values[2]), a3*sin(self.theta_values[2])],
                        [ 0, sin(alpha_values[2]), cos(alpha_values[2]), a3 ],
                        [ 0, 0, 0, 1 ]])
        DH_34 = np.array([[cos(self.theta_values[3]), -sin(self.theta_values[3])*cos(alpha_values[3]), sin(self.theta_values[3])*sin(alpha_values[3]), a4*cos(self.theta_values[3])],
                        [sin(self.theta_values[3]), cos(self.theta_values[3])*cos(alpha_values[3]), -cos(self.theta_values[3])*sin(alpha_values[3]), a4*sin(self.theta_values[3])],
                        [ 0, sin(alpha_values[3]), cos(alpha_values[3]), a4 ],
                        [ 0, 0, 0, 1 ]])
        DH_45 = np.array([[cos(self.theta_values[4]), -sin(self.theta_values[4])*cos(alpha_values[4]), sin(self.theta_values[4])*sin(alpha_values[4]), a5],
                        [sin(self.theta_values[4]), cos(self.theta_values[4])*cos(alpha_values[4]), -cos(self.theta_values[4])*sin(alpha_values[4]), 0],
                        [ 0, sin(alpha_values[4]), cos(alpha_values[4]), 0 ],
                        [ 0, 0, 0, 1 ]])

        DH_05 = DH_01 @ DH_12 @ DH_23 @ DH_34 @ DH_45

        DH_50 = np.linalg.inv(DH_05)
        transposed_point = DH_50*matrix([[point.x], [point.y], [point.z], [1.0]])
        return transposed_point

    def publish(self):
        if  self.changePos:
            self.old_theta1 += self.step_theta1 
            self.old_theta2 += self.step_theta2
            self.old_theta3 += self.step_theta3 
            self.old_theta4 += self.step_theta4 
            self.old_theta5 += self.step_theta5 
            self.step_counter += 1
        if self.step_counter == self.step:
            time.sleep(1)
            if self.item == "CUBE":
                self.item = "DOWN"
            elif self.item == "DOWN":
                self.state = 1
                self.item = "SHEET"
            elif self.item == "SHEET":
                self.item = "LEAVE"
            elif self.item == "LEAVE":
                self.state = 0
                self.cube_x = self.sheet_x
                self.cube_y = self.sheet_y
                self.cube_z = self.sheet_z + 0.01
                self.cube_or_x = self.sheet_or_x
                self.cube_or_y = self.sheet_or_y
                self.cube_or_z = self.sheet_or_z
                self.cube_or_w = self.sheet_or_w
                self.publish_markers("base_link")
                self.item = "RETURN"
            elif self.item == "RETURN":
                self.item = "NONE"
            
            self.get_logger().info(f"item {self.item}")
            self.received = False
            self.isGoing = False
            self.step_counter = 0
            if self.item != "NONE":
                self.goto_item()
            else:
                self.changePos = False

        new_msg = JointState()
        new_msg.name = [
            "rotate_base__base",
            "down_manipulator__rotation_base",
            "up_manipulator_joint__down_manipulator_joint",
            "manipulator_hand_joint__up_manipulator_joint",
            "tool_joint__manipulator_hand_joint"
        ]
        new_msg.header.frame_id = "base_link"
        new_msg.header.stamp.sec = self.get_clock().now().to_msg().sec
        new_msg.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        new_msg.position = [self.old_theta1,
                            self.old_theta2,
                            self.old_theta3,
                            self.old_theta4,
                            self.old_theta5]
        self.publisher.publish(new_msg)

    def gripper_callback(self, pose):
        if self.state == 1:
            self.cube_x = pose.pose.position.x + 0.024
            self.cube_y = pose.pose.position.y
            self.cube_z = pose.pose.position.z
            self.cube_or_x = pose.pose.orientation.x
            self.cube_or_y = pose.pose.orientation.y
            self.cube_or_z = pose.pose.orientation.z
            self.cube_or_w = pose.pose.orientation.w
            self.publish_markers("tool_joint")

    # def state_callback(self, state_data):
    #     self.state = state_data.data
    #     self.get_logger().info(f"state: {self.state}")

    def publish_markers(self, link):
        cube = Marker()
        # cube.header.frame_id = "base_link"
        cube.header.frame_id = link
        # cube.header.stamp = self.get_clock().now().to_msg()
        cube.header.stamp.sec = self.get_clock().now().to_msg().sec
        cube.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        cube.ns = "marker_namespace"
        cube.id = 0
        cube.type = 1
        cube.action = 0
        # cube.lifetime.sec = 0
        # cube.lifetime.nanosec = 0
        cube.frame_locked = True
        # self.get_logger().info("point:" + str(self.cube_position[0])+","+ str(self.cube_position[1])+"," +str(self.cube_position[2]))
        self.get_logger().info(str(self.cube_x))
        cube.pose.position.x = self.cube_x
        cube.pose.position.y = self.cube_y
        cube.pose.position.z = self.cube_z
        cube.pose.orientation.x = self.cube_or_x
        cube.pose.orientation.y = self.cube_or_y
        cube.pose.orientation.z = self.cube_or_z
        cube.pose.orientation.w = self.cube_or_w
        cube.scale.x = 0.02
        cube.scale.y = 0.02
        cube.scale.z = 0.02
        cube.color.a = 1.0
        cube.color.r = 1.0
        cube.color.g = 0.5
        cube.color.b = 0.0


        sheet = Marker()
        sheet.header.frame_id = "base_link"
        # sheet.header.stamp = self.get_clock().now().to_msg()
        sheet.header.stamp.sec = self.get_clock().now().to_msg().sec
        sheet.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        sheet.ns = "marker_namespace"
        sheet.id = 1
        sheet.type = 1
        sheet.action = 0
        # sheet.lifetime.sec = 0
        # sheet.lifetime.nanosec = 0
        sheet.frame_locked = True
        sheet.pose.position.x = self.sheet_x
        sheet.pose.position.y = self.sheet_y
        sheet.pose.position.z = self.sheet_z
        sheet.pose.orientation.x = self.sheet_or_x
        sheet.pose.orientation.y = self.sheet_or_y
        sheet.pose.orientation.z = self.sheet_or_z
        sheet.pose.orientation.w = self.sheet_or_w
        sheet.scale.x = 0.05
        sheet.scale.y = 0.1
        sheet.scale.z = 0.001
        sheet.color.a = 1.0
        sheet.color.r = 1.0
        sheet.color.g = 1.0
        sheet.color.b = 1.0

        mar_arr = MarkerArray()
        mar_arr.markers.append(cube)
        mar_arr.markers.append(sheet)
        self.array_publisher.publish(mar_arr)


def main(arg=None):
    rclpy.init(args=arg)
    node = ReverseKinematicNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
