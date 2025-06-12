#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rb5_ros2_control.srv import NavigatePath  

class TestClient(Node):

    def __init__(self):
        super().__init__('test_navigate_path_client')
        self.cli = self.create_client(NavigatePath, 'navigate_path')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = NavigatePath.Request()
        self.send_waypoints()

    def send_waypoints(self):
        # Test waypoints 
        waypoints = [0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.06, 0.0, 0.0] 

        self.req.waypoints = waypoints

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result() is not None:
            self.get_logger().info(f"Service responded: success = {self.future.result().success}")
        else:
            self.get_logger().error("Service call failed")

def main(args=None):
    rclpy.init(args=args)
    test_client = TestClient()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
