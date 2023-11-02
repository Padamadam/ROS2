import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.action import RotateAbsolute

PI = 3.1415926


class TurtleControlNode(Node):
    def __init__(self):
        super().__init__('turtle_control_node')

        # Get the parameters for the turtle names
        self.declare_parameter('first_turtle_name', 'turtle1')
        self.declare_parameter('second_turtle_name', 'turtle2')
        self.declare_parameter('third_turtle_name', 'turtle3')
        self.first_turtle_name = self.get_parameter('first_turtle_name').value
        self.second_turtle_name = self.get_parameter('second_turtle_name').value
        self.third_turtle_name = self.get_parameter('third_turtle_name').value

        self.state = 0

        # Create an action client for RotateAbsolute
        self.rotate_absolute_client = ActionClient(
            self,
            RotateAbsolute,
            'turtle1/rotate_absolute'
        )

        # Create a service client for Spawn
        self.spawn_client = self.create_client(
            Spawn,
            'spawn'
        )

        self.spawn_request = Spawn.Request()

    def spawn_turtle(self, turtle_name, x, y, theta):
        # Call the Spawn service to spawn a new turtle
        self.spawn_request.name = turtle_name
        self.spawn_request.x = x
        self.spawn_request.y = y
        self.spawn_request.theta = theta
        self.get_logger().info(f'Spawning {turtle_name}...')
        self.spawn_client.call_async(self.spawn_request)

    def rotate_turtle(self, turtle_name, angle):
        goal_msg = RotateAbsolute.Goal()
        goal_msg.theta = angle
        self.get_logger().info(f'Rotating {turtle_name} by {angle} degrees...')
        self.rotate_absolute_client.wait_for_server()
        future = self.rotate_absolute_client.send_goal_async(goal_msg)
        future.add_done_callback(self.execute_callback)

    def execute_callback(self, future):
        self.get_logger().info('Executing goal...')
        result = future.result()
        self._get_result_future = result.get_result_async()
        self._get_result_future.add_done_callback(self.execute_result_callback)

    def execute_result_callback(self, future):
        self.state += 1
        self.run_task()

    def run_task(self):
        if self.state == 0:
            # Rotate the first turtle left
            self.rotate_turtle(self.first_turtle_name, PI)

        if self.state == 1:
            # Spawn the second turtle
            self.spawn_turtle(self.second_turtle_name, 1.0, 1.0, 0.0)

            self.rotate_turtle(self.first_turtle_name, PI/2)

        if self.state == 2:
            self.rotate_turtle(self.first_turtle_name, 0.0)

        if self.state == 3:
            self.rotate_turtle(self.first_turtle_name, -PI/2)

        if self.state == 4:
            # Spawn the third turtle
            self.spawn_turtle(self.third_turtle_name, 1.0, 5.0, 0.0)


def main(arg=None):
    rclpy.init(args=arg)
    node = TurtleControlNode()
    node.run_task()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
