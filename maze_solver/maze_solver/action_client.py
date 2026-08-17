import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from time import sleep

class WallServiceNode(Node):
    def __init__(self):
        super.__init__("wall_service")
        self.client  = self.create_client(SetBool,"/toggle_walls_1_2")
        self.get_logger().info("waiting for service to be available")
        while not self.client.wait_for_service(timeout_sec = 1):
            pass
        self.get_logger().info("service available")
        self.request = SetBool.Request()

    def toggle_walls(self,state):
        self.request.data = state
        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.walls_call_back)

    def walls_call_back(self,future):
        response  = future.result()
        if response.success:
            msg = response.message
            self.get_logger().info(msg)
        else:
            self.get_logger().error("error : walls didn't toggle")

def solve_maze():
    wall_node = WallServiceNode()
    wall_node.toggle_walls(True)
    sleep(1000)
    wall_node.toggle_walls(False)
    wall_node.destroy_node()

def main():
    rclpy.init()
    solve_maze()
    rclpy.shutdown()
