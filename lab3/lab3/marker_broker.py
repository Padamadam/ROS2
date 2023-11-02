import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from ros2_aruco_interfaces.msg import ArucoMarkers

class MarkerBroker(Node):

    def __init__(self):
        super().__init__("MarkerPublisher")
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

        self.sub = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.callback,
            10
        )

        


        # timer_period = 0.5
        # self.timer = self.create_timer(timer_period, self.timer_callback)


    def callback(self, msg):
        self.poses = msg.poses

        self.marker_ids = msg.marker_ids
        self.publish2()


    
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
    


def main(args=None):
    rclpy.init(args=args)
    markerBrokerNode = MarkerBroker()
    rclpy.spin(markerBrokerNode)
    markerBrokerNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

