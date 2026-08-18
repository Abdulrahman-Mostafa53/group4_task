import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from time import sleep

class WallServiceClient(Node):

    def __init__(self):
        super().__init__('wall_service_client')

        self.client = self.create_client(
            SetBool,
            '/toggle_walls_1_2'
        )

        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Waiting for /toggle_walls_1_2 service...'
            )

        self.get_logger().info(
            'Wall service is available!'
        )

    def toggle_walls(self ,state):
        request = SetBool.Request()

        # true = toggle the walls
        request.data = state

        future = self.client.call_async(request)

        future.add_done_callback(self.service_response)

    def service_response(self, future):
        try:
            response = future.result()

            if response.success:
                self.get_logger().info(
                    f'Service succeeded: {response.message}'
                )
            else:
                self.get_logger().error(
                    f'Service failed: {response.message}'
                )

        except Exception as e:
            self.get_logger().error(
                f'Service call failed: {e}'
            )

def solve_maze():
    wall_node = WallServiceClient()
    wall_node.toggle_walls(True)
    sleep(1)
    wall_node.toggle_walls(False)
    sleep(1)
    wall_node.toggle_walls(True)
    sleep(1)
    wall_node.toggle_walls(False)
    sleep(1)
    wall_node.toggle_walls(True)
    sleep(1)
    wall_node.toggle_walls(False)
    sleep(1)
    wall_node.toggle_walls(True)
    sleep(1)
    wall_node.toggle_walls(False)
    sleep(1)
    wall_node.destroy_node()

def main():
    rclpy.init()
    solve_maze()
    rclpy.shutdown()
