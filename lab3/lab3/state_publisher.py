import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState




class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')
        self.subscription = self.create_subscription(
            JointState,
            "/dobot_joint_states",
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(
            JointState, 
            "/joint_states", 
            10
        )
        
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))


    def listener_callback(self, msg: JointState):
        t1, t2, t3, t4 = msg.position
        pos = [t1 , t2, t3-t2 + 1.57, -t3+1.57, -t4] 
        self.timer_callback(pos)


    def timer_callback(self, pos):
        msg = JointState()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "base_link"
        msg.name = ['shoulder_joint', 'elbow_wheel_joint', 'forearm_wheel_joint', 'hand_joint', 'device_joint']
        msg.position = pos
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    rclpy.spin(node)

    
if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState




# class StatePublisher(Node):

#     def __init__(self):
#         super().__init__('state_publisher')
#         self.subscription = self.create_subscription(
#             JointState,
#             "/dobot_joint_states",
#             self.listener_callback,
#             10
#         )
#         self.publisher = self.create_publisher(
#             JointState, 
#             "/joint_states", 
#             10
#         )
        
#         self.nodeName = self.get_name()
#         self.get_logger().info("{0} started".format(self.nodeName))


#     def listener_callback(self, msg: JointState):
#         t1, t2, t3, t4 = msg.position
#         pos = [t1 , t2, t3-t2 + 1.57, -t3+1.57, -t4] 
#         self.timer_callback(pos)


#     def timer_callback(self, pos):
#         msg = JointState()
#         now = self.get_clock().now()
#         msg.header.stamp = now.to_msg()
#         msg.header.frame_id = "base_link"
#         msg.name = ['shoulder_joint', 'elbow_wheel_joint', 'forearm_wheel_joint', 'hand_joint', 'device_joint']
#         msg.position = pos
#         self.publisher.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = StatePublisher()
#     rclpy.spin(node)

    
# if __name__ == '__main__':
#     main()
