import rclpy
from visualization_msgs.msg import Marker
#from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.node import Node
from math import cos, sin, atan2
import math

class Marker_Node(Node):
    def __init__(self):
        super().__init__('Marker')
        self.publisher = self.create_publisher(Marker, 'marker_publisher', 10)
        self.markers_subscription = self.create_subscription(PoseArray,'/camera_link',self.markers_callback,10)
        self.position_subscription = self.create_subscription(PoseStamped,'/curent_position',self.position_callback,10)
        self.gripper_subscription = self.create_subscription(Quaternion,'/gripper_pose', self.gripper_callback,10)
        self.cam_pos = [0.0, 0.0, 0.0]
        self.cam_rot = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.cube_pos = [0.0, 0.0, 0.0]
        self.cube_rot = 0.0
        self.sheet_pos = [0.0, 0.0, 0.0]
        self.sheet_rot = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.cube = Marker()
        self.paper_sheet = Marker()
        # self.timer = self.create_timer(.1, self.publish)
        self.config_cube()
        self.config_paper_sheet()
   
   
    def config_cube(self):
        self.cube.header.frame_id = "base"
        q = self.cam_rot
        cam_angle = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        rot = self.euler_to_quaternion(0, 0, cam_angle + self.cube_rot)

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        self.cube.type = 1
        self.cube.id = 1

        # Set the scale of the marker
        self.cube.scale.x = 0.02
        self.cube.scale.y = 0.02
        self.cube.scale.z = 0.02

        # Set the color
        self.cube.color.r = 0.0
        self.cube.color.g = 1.0
        self.cube.color.b = 0.0
        self.cube.color.a = 1.0
        self.cube.pose.position.x = self.cam_pos[0] + self.cube_pos[0] * sin(cam_angle) - self.cube_pos[1] * cos(cam_angle)
        self.cube.pose.position.y = self.cam_pos[1] - self.cube_pos[1] * sin(cam_angle) - self.cube_pos[0] * cos(cam_angle)
        self.cube.pose.position.z = self.cam_pos[2] - self.cube_pos[2]
        self.cube.pose.orientation.x = rot.x
        self.cube.pose.orientation.y = rot.y
        self.cube.pose.orientation.z = rot.z
        self.cube.pose.orientation.w = rot.w

    def config_cube2(self):
        self.cube.header.frame_id = "base"
        q = self.cam_rot
        cam_angle = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        rot = self.euler_to_quaternion(0, 0, cam_angle + self.cube_rot)

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        self.cube.type = 1
        self.cube.id = 1

        # Set the scale of the marker
        self.cube.scale.x = 0.02
        self.cube.scale.y = 0.02
        self.cube.scale.z = 0.02

        # Set the color
        self.cube.color.r = 0.0
        self.cube.color.g = 1.0
        self.cube.color.b = 0.0
        self.cube.color.a = 1.0
        self.cube.pose.position.x = self.cube_pos[0]
        self.cube.pose.position.y = self.cube_pos[1]
        self.cube.pose.position.z = self.cube_pos[2] + 0.01
        self.cube.pose.orientation.x = rot.x
        self.cube.pose.orientation.y = rot.y
        self.cube.pose.orientation.z = rot.z
        self.cube.pose.orientation.w = rot.w

    def config_paper_sheet(self):
        self.paper_sheet.header.frame_id = "base"
        q = self.sheet_rot
        sheet_angle = -atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        q = self.cam_rot
        cam_angle = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        rot = self.euler_to_quaternion(0, 0, cam_angle + sheet_angle)

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        self.paper_sheet.type = 1
        self.paper_sheet.id = 2

        # Set the scale of the marker
        self.paper_sheet.scale.x = 0.05
        self.paper_sheet.scale.y = 0.1
        self.paper_sheet.scale.z = 0.001

        # Set the color
        self.paper_sheet.color.r = 1.0
        self.paper_sheet.color.g = 1.0
        self.paper_sheet.color.b = 1.0
        self.paper_sheet.color.a = 1.0
        self.paper_sheet.pose.position.x = self.cam_pos[0] + self.sheet_pos[0] * sin(cam_angle) - self.sheet_pos[1] * cos(cam_angle)
        self.paper_sheet.pose.position.y = self.cam_pos[1] - self.sheet_pos[1] * sin(cam_angle) - self.sheet_pos[0] * cos(cam_angle)
        self.paper_sheet.pose.position.z = self.cam_pos[2] - self.sheet_pos[2]
        self.paper_sheet.pose.orientation.x = rot.x
        self.paper_sheet.pose.orientation.y = rot.y
        self.paper_sheet.pose.orientation.z = rot.z
        self.paper_sheet.pose.orientation.w = rot.w
    
    
    def markers_callback(self, msg):
        print('marker callback')
        for id, pose in zip([1, 2], msg.poses):
            if(id == 1):
                self.sheet_pos = [pose.position.x, pose.position.y, pose.position.z]
                self.sheet_rot = Quaternion(x=pose.orientation.x, y=pose.orientation.y, z=pose.orientation.z, w=pose.orientation.w)
            else:
                self.cube_pos = [pose.position.x, pose.position.y, pose.position.z]
                q = Quaternion(x=pose.orientation.x, y=pose.orientation.y, z=pose.orientation.z, w=pose.orientation.w)
                self.cube_rot = -atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.publish()

    def position_callback(self, msg):
        self.cam_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.cam_rot = Quaternion(x=msg.pose.orientation.x, y=msg.pose.orientation.y, z=msg.pose.orientation.z, w=msg.pose.orientation.w)

    def gripper_callback(self, msg):
        self.cube_pos = [msg.x, msg.y, msg.z]
        self.cube_rot = msg.w
        self.config_cube2()
        self.publisher.publish(self.cube)

    def publish(self):
        self.config_paper_sheet()
        self.config_cube()
        self.publisher.publish(self.cube)
        self.publisher.publish(self.paper_sheet)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Marker_Node()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '_main_':
    main()
