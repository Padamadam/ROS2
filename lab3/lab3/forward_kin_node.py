import rclpy
from rclpy.node import Node
from math import sin, cos, sqrt
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import JointState
import numpy as np


class ForwardKinNode(Node):
    def __init__(self):
        super().__init__('forward_kin_node')
        # publikowanie położenia końcówki manipulatora
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/pose',
            10
            )

        # pobieram informacje o położeniu manipulatora (położenie jointów)
        self.joints_subscribtion = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )
        self.elbow_length = 0.135
        self.forearm_length = 0.147
        self.base_height = 0.038 + 0.1 + 0.019 / 2
        self.device_length = 0.11

        self.get_logger().info("Node started")
        self.position = [0, 0, 0, 0]
        self.joint_states = []
        self.pose_stamped = PoseStamped()
        self.joint_state_msg = JointState()




    def forward_kinematic(self, theta_values):
        translation = self.geometry_method(theta_values)
        rotation = self.orient(theta_values)
        return translation, rotation



    def orient(self, theta_values):
        a1 = self.base_height
        a2 = self.elbow_length
        a3 = self.forearm_length
        a4 = 0 
        a5 = 0 
        efec_len = self.device_length

        alpha_values = [np.deg2rad(-90), 0, 0 ,np.deg2rad(90), 0]
        
        DH_01 = np.array([[cos(theta_values[0]), -sin(theta_values[0])*cos(alpha_values[0]), sin(theta_values[0])*sin(alpha_values[0]), a1*cos(theta_values[0])],
                        [sin(theta_values[0]), cos(theta_values[0])*cos(alpha_values[0]), -cos(theta_values[0])*sin(alpha_values[0]), a1*sin(theta_values[0])],
                        [ 0, sin(alpha_values[0]), cos(alpha_values[0]), a1 ],
                        [ 0, 0, 0, 1 ]])
        DH_12 = np.array([[cos(theta_values[1]), -sin(theta_values[1])*cos(alpha_values[1]), sin(theta_values[1])*sin(alpha_values[1]), a2*cos(theta_values[1])],
                        [sin(theta_values[1]), cos(theta_values[1])*cos(alpha_values[1]), -cos(theta_values[1])*sin(alpha_values[1]), a2*sin(theta_values[1])],
                        [ 0, sin(alpha_values[1]), cos(alpha_values[1]), a2 ],
                        [ 0, 0, 0, 1 ]])
        DH_23 = np.array([[cos(theta_values[2]), -sin(theta_values[2])*cos(alpha_values[2]), sin(theta_values[2])*sin(alpha_values[2]), a3*cos(theta_values[2])],
                        [sin(theta_values[2]), cos(theta_values[2])*cos(alpha_values[2]), -cos(theta_values[2])*sin(alpha_values[2]), a3*sin(theta_values[2])],
                        [ 0, sin(alpha_values[2]), cos(alpha_values[2]), a3 ],
                        [ 0, 0, 0, 1 ]])
        DH_34 = np.array([[cos(theta_values[3]), -sin(theta_values[3])*cos(alpha_values[3]), sin(theta_values[3])*sin(alpha_values[3]), a4*cos(theta_values[3])],
                        [sin(theta_values[3]), cos(theta_values[3])*cos(alpha_values[3]), -cos(theta_values[3])*sin(alpha_values[3]), a4*sin(theta_values[3])],
                        [ 0, sin(alpha_values[3]), cos(alpha_values[3]), a4 ],
                        [ 0, 0, 0, 1 ]])
        DH_45 = np.array([[cos(theta_values[4]), -sin(theta_values[4])*cos(alpha_values[4]), sin(theta_values[4])*sin(alpha_values[4]), a5*cos(theta_values[4])],
                        [sin(theta_values[4]), cos(theta_values[4])*cos(alpha_values[4]), -cos(theta_values[4])*sin(alpha_values[4]), a5*sin(theta_values[4])],
                        [ 0, sin(alpha_values[4]), cos(alpha_values[4]), a5 ],
                        [ 0, 0, 0, 1 ]])
        
        DH_05 = DH_01 @ DH_12 @ DH_23 @ DH_34 @ DH_45

        rotation = Quaternion()

        tr = DH_05[0,0] + DH_05[1, 1] + DH_05[2, 2]
        w = np.sqrt(max(0, tr + 1)) / 2
        rotation.x = (DH_05[2, 1] - DH_05[1, 2]) / (4 * w)
        rotation.y = (DH_05[0, 2] - DH_05[2, 0]) / (4 * w)
        rotation.z = (DH_05[1, 0] - DH_05[0, 1]) / (4 * w)
        rotation.w = np.sqrt(max(0, tr + 1)) / 2

        return rotation

    

    def geometry_method(self, theta_values):
        shoulder_value = theta_values[0]
        elbow_value = theta_values[1]
        forearm_value = theta_values[2]
        device_value = theta_values[3]

        phi0 = elbow_value
        phi1 = elbow_value + forearm_value
        phi2 = elbow_value + forearm_value + device_value

        Xstat = sin(phi0) * self.elbow_length + sin(phi1) * self.forearm_length + sin(phi2) * self.device_length 
        Zstat = cos(phi0) * self.elbow_length + cos(phi1) * self.forearm_length + cos(phi2) * self.device_length

        Xdyn = Xstat * cos(shoulder_value)
        Ydyn = Xstat * sin(shoulder_value)
        Zdyn = Zstat + 0.119 

        translation = [Xdyn, Ydyn, Zdyn, 0]

        return translation
    


    # publikowanie położenia końcówki manipulatora
    def joint_callback(self, msg):
        self.joint_states = msg.position
        translation, rotation = self.forward_kinematic(self.joint_states)
        self.create_pose(translation, rotation)
        self.pose_publisher.publish(self.pose_stamped)


    def create_pose(self, translation, rotation):
        self.pose_stamped.header.frame_id = 'base_link'
        self.pose_stamped.pose.position.x = translation[0]
        self.pose_stamped.pose.position.y = translation[1]
        self.pose_stamped.pose.position.z = translation[2]
        self.pose_stamped.pose.orientation = rotation



def main(args=None):
    rclpy.init(args=args)
    viz_publisher = ForwardKinNode()
    rclpy.spin(viz_publisher)
    viz_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()