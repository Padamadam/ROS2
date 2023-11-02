import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from math import cos, sin, sqrt
from geometry_msgs.msg import Pose

import numpy as np

class PickSim(Node):
    def __init__(self):
        super().__init__('pick_sim')

        self.sub_cube = self.create_subscription(
            Marker,
            '/marker',
            self.callback_marker,
            10
        )
        
        self.pub = self.create_publisher(
            JointState, 
            "/joint_states", 
            10
        )

     
        self.marker_pose = self.create_publisher(
            Marker, 
            "/marker_pose", 
            10
        )

        self.elbow_length = 0.135
        self.forearm_length = 0.147
        self.base_height = 0.038 + 0.1 + 0.019 / 2
        self.device_length = 0.05

        self.cube_pos = [0.0, 0.0, 0.0, 0.0]
        self.sheet_pos = [0.0, 0.0, 0.0, 0.0]
        self.theta_values = [0.0, 0.507, 1.057, 1.578, 0.0]



    def callback_marker(self, msg):
        # self.get_logger().info(f'Point: {msg.points}')
        # pos = InverseKinNode.inverse_kinematics(msg)
        if msg.id == 1:
            self.cube_pos = self.transpose_from_camera(msg.pose.position)
            msg.pose.position.x = float(np.matrix.flatten(self.cube_pos[0]))
            msg.pose.position.y = float(np.matrix.flatten(self.cube_pos[1]))
            msg.pose.position.z = float(np.matrix.flatten(self.cube_pos[2]))
            self.marker_pose.publish(msg)
        else:
            self.sheet_pos = self.transpose_from_camera(msg.pose.position)
            msg.pose.position.x = float(np.matrix.flatten(self.sheet_pos[0]))
            msg.pose.position.y = float(np.matrix.flatten(self.sheet_pos[1]))
            msg.pose.position.z = float(np.matrix.flatten(self.sheet_pos[2]))
            self.marker_pose.publish(msg)


    def transpose_from_camera(self, point):
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
        transposed_point = DH_50*np.matrix([[point.x], [point.y], [point.z], [1.0]])
        return transposed_point

def main(args=None):
    rclpy.init(args=args)
    pick_node = PickSim()
    rclpy.spin(pick_node)
    pick_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()