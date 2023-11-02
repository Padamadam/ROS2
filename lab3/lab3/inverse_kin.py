import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import numpy as np
from math import sin, cos, sqrt, atan, acos, pi, atan2, tan, asin



class InverseKinNode(Node):
    def __init__(self):
        super().__init__('inverse_kin_node')

        # pobieram informacje o położeniu manipulatora (położenie jointów)
        self.publisher = self.create_publisher(
            JointState, 
            'joint_states', 
            10)
        
        # publikuję płynne przejście kostki
        self.cube_pub = self.create_publisher(
            Marker,
            '/marker',
            10
        )
        # pobieram informacje o położeniu testowych markerów
        self.sub = self.create_subscription(
            Marker,
            '/marker',
            self.point_callback,
            10
        )

        # odczytuję połozenie końcówki manipulatora
        self.sub_dev_pose = self.create_subscription(
            PoseStamped,
            '/pose',
            self.dev_pose,
            10
        )


        self.theta1 = 0.0
        self.theta2 = 0.507
        self.theta3 = 1.057
        self.theta4 = 1.578
        self.theta5 = 0.0

        self.old_theta1 = 0.0
        self.old_theta2 = 0.507
        self.old_theta3 = 1.057
        self.old_theta4 = 1.578
        self.old_theta5 = 0.0

        self.step = 200
        self.step_counter = 0

        self.elbow_length = 0.135
        self.forearm_length = 0.147
        self.base_height = 0.038 + 0.1 + 0.019 / 2
        self.device_length = 0.11      
        self.base_height = 0.038
        self.shoulder_length = 0.1
        self.hand_width = 0.01
        self.camera_length = 0.05

        self.dh = self.elbow_length
        self.uh = self.forearm_length

        self.changePos = False

        self.step_theta1 = 0.0
        self.step_theta2 = 0.0
        self.step_theta3 = 0.0
        self.step_theta4 = 0.0
        self.step_theta5 = 0.0
        
        self.base = self.base_height + self.shoulder_length
        self.l2 = self.elbow_length
        self.l3 = self.forearm_length
        
        self.get_logger().info("InverseKin Node started")
        
        self.joint_states = []
        self.joint_header = []
        self.pose_stamped = PoseStamped()
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.init_pos()
        self.point_setted = True
        self.item = "NONE"



    # odczytuje położenie końcówki
    def dev_pose(self, msg):
        self.dev_x = msg.pose.position.x
        self.dev_y = msg.pose.position.y
        self.dev_z = msg.pose.position.z
        self.dev_oz = msg.pose.orientation.z

    
    def goto_item(self):
        if self.item == "CUBE":
            x = self.cube_x
            y = self.cube_y
            z = self.cube_z + 0.1
            or_z = self.cube_or_z
            or_w = self.cube_or_w
        if self.item == "DOWN":
            x = self.cube_x
            y = self.cube_y
            z = self.cube_z - 0.013 + 0.1
            or_z = self.cube_or_z
            or_w = self.cube_or_w
        if self.item == "SHEET":
            x = self.sheet_x
            y = self.sheet_y
            z = self.sheet_z + 0.05 + 0.1
            or_z = self.cube_or_z
            or_w = self.sheet_or_w
        if self.item == "LEAVE":
            x = self.sheet_x
            y = self.sheet_y
            z = self.sheet_z + 0.1
            or_z = self.sheet_or_z
            or_w = self.sheet_or_w
        if self.item == "RETURN":
            x = self.sheet_x
            y = self.sheet_y
            z = self.sheet_z + 0.1 + 0.1
            or_z = self.sheet_or_z
            or_w = self.sheet_or_w

        # offset aby manipulator nie obniżył się zbyt bardzo 
        z = z - 0.079

        # wyznaczenie i przypisanie położeń jointów
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
            hand_joint= - alfa - beta + pi
            device_joint = asin(or_z) + shoulder_joint

            # device_joint =  shoulder_joint
            self.get_logger().info(f'shoulder_joint: {round(shoulder_joint,3)}')
            self.get_logger().info(f'elbow_wheel_joint: {round(elbow_wheel_joint,3)}')
            self.get_logger().info(f'forearm_wheel_joint: {round(forearm_wheel_joint,3)}')
            self.get_logger().info(f'hand_joint: {round(hand_joint,3)}')
            self.get_logger().info(f'device_joint: {round(device_joint,3)}')
        except:
            self.get_logger().info("Out of range.")
            shoulder_joint = 0.0
            elbow_wheel_joint = 0.507
            forearm_wheel_joint = 1.057
            hand_joint = 1.578
            device_joint = 0.0



        self.theta1 = shoulder_joint
        self.theta2 = elbow_wheel_joint
        self.theta3 = forearm_wheel_joint
        self.theta4 = hand_joint
        self.theta5 = device_joint

        self.step_theta1 = (self.theta1 - self.old_theta1)/self.step
        self.step_theta2 = (self.theta2 - self.old_theta2)/self.step
        self.step_theta3 = (self.theta3 - self.old_theta3)/self.step
        self.step_theta4 = (self.theta4 - self.old_theta4)/self.step
        self.step_theta5 = (self.theta5 - self.old_theta5)/self.step
        self.changePos = True


        self.step_theta1 = (self.theta1 - self.old_theta1)/self.step
        self.step_theta2 = (self.theta2 - self.old_theta2)/self.step
        self.step_theta3 = (self.theta3 - self.old_theta3)/self.step
        self.step_theta4 = (self.theta4 - self.old_theta4)/self.step
        self.step_theta5 = (self.theta5 - self.old_theta5)/self.step
        self.changePos = True



    def point_callback(self, msg):
        # self.get_logger().info(f'x : {round(msg.point.x, 3)}, y : {round(msg.point.y, 3)}, z : {round(msg.point.z, 3)}')
        if msg.id == 1:
            self.cube_x = msg.pose.position.x
            self.cube_y = msg.pose.position.y
            self.cube_z = msg.pose.position.z
            self.cube_or_z = msg.pose.orientation.z
            self.cube_or_w = msg.pose.orientation.w
        else:
            self.sheet_x = msg.pose.position.x
            self.sheet_y = msg.pose.position.y
            self.sheet_z = msg.pose.position.z
            self.sheet_or_z = msg.pose.orientation.z
            self.sheet_or_w = msg.pose.orientation.w
            self.point_setted = True
            self.item = "CUBE"

            self.goto_item()




    def create_marker(self):
        cube_marker = Marker()
        cube_marker.header.frame_id = "base_link"

        cube_marker.type = 1
        cube_marker.id = 1

        cube_marker.pose.position.x = self.cube_x
        cube_marker.pose.position.y = self.cube_y
        cube_marker.pose.position.z = self.cube_z

        cube_marker.pose.orientation.x = 0.0
        cube_marker.pose.orientation.y = 0.0
        cube_marker.pose.orientation.z = -self.cube_or_z
        cube_marker.pose.orientation.w = self.cube_or_w

        cube_marker.scale.x = 0.02
        cube_marker.scale.y = 0.02
        cube_marker.scale.z = 0.02

        cube_marker.color.a = 1.0
        cube_marker.color.r = 1.0
        cube_marker.color.g = 0.6
        cube_marker.color.b = 0.0
        return cube_marker



    def timer_callback(self):
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
                self.cube_or_z = self.sheet_or_z
                self.cube_or_w = self.sheet_or_w
                cube_marker = self.create_marker()
                self.cube_pub.publish(cube_marker)
                self.item = "RETURN"
            elif self.item == "RETURN":
                self.item = "NONE"
            
            self.get_logger().info(f"item {self.item}")
            self.step_counter = 0
            if self.item != "NONE":
                self.goto_item()
            else:
                self.changePos = False


        new_msg = JointState()
        new_msg.name = [
            "shoulder_joint",
            "elbow_wheel_joint",
            "forearm_wheel_joint",
            "hand_joint",
            "device_joint"
        ]
        new_msg.header.frame_id = "base_link"
        new_msg.header.stamp.sec = self.get_clock().now().to_msg().sec
        new_msg.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        position = [self.old_theta1,
                    self.old_theta2,
                    self.old_theta3,
                    self.old_theta4,
                    self.old_theta5]
        for i in range(len(position)):
            new_msg.position.append(position[i])
        self.publisher.publish(new_msg)
        # self.publisher.publish()
            
        if self.item == 'SHEET' or self.item == 'LEAVE':
            self.cube_x = self.dev_x
            self.cube_y = self.dev_y
            self.cube_z = self.dev_z
            self.cube_or_z = sin(self.old_theta5)
            self.cube_or_w = cos(self.old_theta5)
            cube_marker = self.create_marker()
            self.cube_pub.publish(cube_marker)




def main(args=None):
    rclpy.init(args=args)
    inv_kin = InverseKinNode()
    rclpy.spin(inv_kin)
    inv_kin.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()