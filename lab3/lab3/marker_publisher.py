from math import cos, sin, sqrt
import random
import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState


class MarkerPublisher(Node):

    def __init__(self):
        # publikuję na temacie /marker markery - kostkę i kartkę 
        super().__init__("MarkerPublisher")
        self.marker_publisher = self.create_publisher(
            Marker,
            "/marker",
            10
        )

        # pobieram informacje o położeniu manipulatora (położenie jointów)
        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )
        
        self.base_height = 0.038
        self.shoulder_length = 0.1
        self.elbow_length = 0.135
        self.forearm_length = 0.147
        self.hand_width = 0.01
        self.device_length = 0.05

        self.theta_values = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.cube_position = [0.0, -0.04, -0.03]
        self.cube_orientation = [0.0, 0.0, 0.0, 1.0]
        self.sheet_position = [0.2, -0.03, -0.04]
        self.sheet_orientation = [0.0, 0.0, 0.0, 1.0]

        # nowe testy co 20s
        timer_period = 20
        self.timer = self.create_timer(timer_period, self.point_test)




    # przypisuję aktualne położenia jointów manipulatora do theta_values
    def joint_callback(self, msg):
        self.theta_values = msg.position


    # publikowanie testowych markerów
    def point_test(self):
        self.get_logger().info("Test")
        # losowanie pozycji kostki i kartki (w ukladzie świata?)

        # ---> współrzędne x
        self.sheet_pos_test_x = random.uniform(0.20, 0.26) 
        self.cube_pos_test_x = random.uniform(0.20, 0.26) 
        # ---> współrzędne y
        self.sheet_pos_test_y =random.uniform(-0.15, 0.15)
        self.cube_pos_test_y =random.uniform(-0.15, 0.15)
        # ---> obrót wokół osi z
        self.cube_angle_test = random.uniform(-1, 1)
        self.sheet_angle_test = random.uniform(-1, 1)


        # przypisanie wylosowanych pozycji i orientacji
        self.sheet_position = [self.sheet_pos_test_x, self.sheet_pos_test_y, 0]
        self.sheet_orientation = [0.0, 0.0, sqrt(1-self.sheet_angle_test**2), self.sheet_angle_test]
    
        self.cube_orientation = [0.0, 0.0, self.cube_angle_test, sqrt(1-self.cube_angle_test**2)]
        self.cube_position = [self.cube_pos_test_x, self.cube_pos_test_y, 0.01]


        # przypisanie pozycji po przetransponowaniu do układu kamery
        self.cube_position = self.transpose(self.cube_position)
        self.sheet_position = self.transpose(self.sheet_position)
        self.timer_callback()

        



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

        # DH_50 = np.linalg.inv(DH_05)
        transposed_point = DH_05*np.matrix([[point[0]], [point[1]], [point[2]], [1.0]])
        return transposed_point



    def timer_callback(self):

        cube_marker = Marker()
        cube_marker.header.frame_id = "base_link"
        cube_marker.type = 1
        cube_marker.id = 1

        cube_marker.pose.position.x = self.cube_pos_test_x
        cube_marker.pose.position.y = self.cube_pos_test_y
        cube_marker.pose.position.z = 0.04

        cube_marker.pose.orientation.x = float(self.cube_orientation[0])
        cube_marker.pose.orientation.y = float(self.cube_orientation[1])
        cube_marker.pose.orientation.z = float(self.cube_orientation[2])
        cube_marker.pose.orientation.w = float(self.cube_orientation[3])

        cube_marker.scale.x = 0.02
        cube_marker.scale.y = 0.02
        cube_marker.scale.z = 0.02

        cube_marker.color.a = 1.0
        cube_marker.color.r = 1.0
        cube_marker.color.g = 0.6
        cube_marker.color.b = 0.0

        paper_sheet_marker = Marker()
        paper_sheet_marker.header.frame_id = "base_link"
        paper_sheet_marker.type = 1
        paper_sheet_marker.id = 2

        paper_sheet_marker.pose.position.x = self.sheet_pos_test_x
        paper_sheet_marker.pose.position.y = self.sheet_pos_test_y
        paper_sheet_marker.pose.position.z = 0.03

        paper_sheet_marker.pose.orientation.x = float(self.sheet_orientation[0])
        paper_sheet_marker.pose.orientation.y = float(self.sheet_orientation[1])
        paper_sheet_marker.pose.orientation.z = float(self.sheet_orientation[2])
        paper_sheet_marker.pose.orientation.w = float(self.sheet_orientation[3])

        paper_sheet_marker.scale.x = 0.05
        paper_sheet_marker.scale.y = 0.1
        paper_sheet_marker.scale.z = 0.001

        paper_sheet_marker.color.a = 1.0
        paper_sheet_marker.color.r = 1.0
        paper_sheet_marker.color.g = 1.0
        paper_sheet_marker.color.b = 1.0

        self.marker_publisher.publish(cube_marker)
        self.marker_publisher.publish(paper_sheet_marker)




def main(args=None):
    rclpy.init(args=args)
    markerPublisherNode = MarkerPublisher()
    rclpy.spin(markerPublisherNode)
    markerPublisherNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

