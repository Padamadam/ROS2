import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import GripperControl
import time


CUBE_WIDTH = 21


class PickNode(Node):
    def __init__(self):
        super().__init__('pick_node')

        self.declare_parameter('height', 0)
        self.height =self.get_parameter('height').value

        self.pick_client = ActionClient(
            self, 
            PointToPoint,
            '/PTP_action'
        )

        self.grip_client = self.create_client(
            GripperControl,
            'dobot_gripper_service'
        )

        self.cube_number = 1
        self.state = 0


    def set_grip(self, goal, isRunning):
        self.grip_request = GripperControl.Request()
        self.grip_request.gripper_state = goal
        if goal == 'open':
            self.get_logger().info('Opening...')
        elif goal == 'close':
            self.get_logger().info('Closing...')
        self.grip_request.keep_compressor_running = isRunning
        self.future = self.grip_client.call_async(self.grip_request)


    def move(self, point):
        goal_point = PointToPoint.Goal()
        goal_point.motion_type = 1
        goal_point.target_pose = point
        goal_point.velocity_ratio = 0.5
        goal_point.acceleration_ratio = 0.5
        self.pick_client.wait_for_server()

        future = self.pick_client.send_goal_async(goal_point)
        future.add_done_callback(self.execute_callback)


    def execute_callback(self, future):
        self.get_logger().info('Executing goal...')
        result = future.result()
        self._get_result_future = result.get_result_async()
        self._get_result_future.add_done_callback(self.execute_result_callback)


    def execute_result_callback(self, future):
        if self.state == 7:
            self.state = 0
        else:
            self.state += 1
        self.run_task()
    

    def run_task(self):
        iter_delta = (self.cube_number - 1) * CUBE_WIDTH
        put_z = 0.0 + iter_delta 
       
        if self.cube_number <= self.height:
            if self.state == 0:
                self.set_grip("open", True)
                self.move([198.4, -45.7, 46.3, -12.9])

            elif self.state == 1:
                self.move([193.2, -46.9, 1.5, -13.6])

            elif self.state == 2:
                self.set_grip("close", True)
                self.move([193.2, -46.9, 1.5, -13.6])

            elif self.state == 3:
                self.move([198.4, -45.7, 46.3, -12.9])
            
            elif self.state == 4:
                self.move([224.3, 60.4, 100.4, 15.1])

            elif self.state == 5:
                self.move([218.6, 71.0, put_z, 18.0])

            elif self.state == 6:
                self.set_grip("open", False)
                self.move([218.6, 71.0, 10.9, 18.0])

            elif self.state == 7:
                self.cube_number += 1
                self.move([224.3, 60.4, 100.4, 15.1])

            
def main(arg=None):
    rclpy.init(args=arg)
    node = PickNode()
    node.run_task()
    rclpy.spin(node)


if __name__ == "__main__":
    main()


