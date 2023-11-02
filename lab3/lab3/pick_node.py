import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from ros2_aruco_interfaces.msg import ArucoMarkers
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from dobot_msgs.action import PointToPoint

class PickNode(Node):
    def __init__(self):
        super().__init__('state_publisher')

        self.sub_cube = self.create_subscription(
            Marker,
            '/marker_cube',
            self.callback_cube,
            10
        )

        self.sub_sheet = self.create_subscription(
            Marker,
            '/marker_cube',
            self.callback_sheet,
            10
        )
        
        self.pub = self.create_publisher(
            JointState, 
            "/joint_states", 
            10
        )

        self.action = ActionClient(
            self,
            PointToPoint,
            '/PTP_action'
        )
        
    def send_point_goal(self, msg):
        goal_msg = PointToPoint.Goal()
        goal_msg.motion_type = 1

        x = msg.pose.position.x * 1000
        y = msg.pose.position.y * 1000 
        z = msg.pose.position.z * 1000

        ox = msg.pose.orientation.x
        oy = msg.pose.orientation.y
        oz = msg.pose.orientation.z
        ow = msg.pose.orientation.w

        goal_msg.target_pose = [x, y, z, 0.0]
        goal_msg.velocity_ratio = 0,5
        goal_msg.acceleration_ratio = 0.3

        self.action.wait_for_server()
        self.action.send_goal_async(goal_msg)

    def callback_cube(self, msg):
        self.get_logger().info(f'Point: {msg.point}')
        self.send_point_goal(msg)


    def callback_sheet(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    pick_node = PickNode()
    rclpy.spin(pick_node)
    pick_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        