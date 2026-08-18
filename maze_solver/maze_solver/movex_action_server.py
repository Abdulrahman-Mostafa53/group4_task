import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from my_robot_interfaces.action import MoveX
from my_robot_interfaces.srv import Error


# Define and initialize the MoveX Action Server node class
class MoveXActionServer(Node):

    # Constructor method to initialize node, ROS 2 communication interfaces, and state variables
    def __init__(self):
        super().__init__('move_x_action_server')

        # Create the Action Server for handling linear movement goals
        self._action_server = ActionServer(
            self,
            MoveX,
            'move_x',
            self.execute_callback
        )

        # Create Publisher to send velocity commands to the robot
        self.publisher_ = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )

        # Create Subscriber to continuously monitor robot odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Variables to track the robot's current position on the X axis
        self.current_x = 0.0
        self.current_y = 0.0
        
        # State variable to track if an emergency stop or external service stop was triggered
        self.is_stopped_via_service = False
        
        # Create Service Server to handle manual stop requests
        self.srv = self.create_service(
            Error,
            'stop_robot',
            self.stop_robot_callback
        )
        
        self.get_logger().info('MoveX Action Server & Stop Robot Service initialized.')

    # Helper function to stop the robot motion immediately without messing up odometry tracking
    def execute_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        self.publisher_.publish(twist)

    # Service callback function to process incoming emergency stop requests
    def stop_robot_callback(self, request, response):
        if request.stop:
            self.is_stopped_via_service = True
            self.execute_stop()
            self.get_logger().info('Robot stopped via service successfully!')
            
            response.success = True
            response.message = 'Robot stopped successfully.'
        else:
            self.get_logger().info('Received stop request as False, doing nothing.')
            response.success = False
            response.message = 'Stop request was False, no action taken.'

        return response

    # Odometry callback function to continuously update current X position
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

   # Main action server callback function to execute linear movement towards target distance
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal: Moving forward...')
        
        # Reset service stop flag at the beginning of a new goal execution
        self.is_stopped_via_service = False
        
        target_distance = goal_handle.request.target_distance    
        speed = 1.0  
        
        start_x = self.current_x
        start_y = self.current_y
        
        feedback_msg = MoveX.Feedback()
        twist = Twist()
        twist.linear.x = speed

        distance_traveled = 0.0

        # Control loop: Keep moving while node is active, target distance not reached, and no service stop requested
        while rclpy.ok() and (distance_traveled < target_distance) and (not self.is_stopped_via_service):
            distance_traveled = math.sqrt((self.current_x - start_x) ** 2 + (self.current_y - start_y) ** 2)

            feedback_msg.current_distance_traveled = distance_traveled
            goal_handle.publish_feedback(feedback_msg)

            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled.')
                self.execute_stop()
                
                goal_handle.canceled()
                result = MoveX.Result()
                result.success = False
                result.final_distance = distance_traveled
                return result

            self.publisher_.publish(twist)
            time.sleep(0.05)

        # Check if the loop was interrupted by the emergency stop service
        if self.is_stopped_via_service:
            self.execute_stop()
            self.get_logger().info(f'Execution aborted via stop service! Traveled: {distance_traveled:.2f}m')
            
            goal_handle.abort()
            result = MoveX.Result()
            result.success = False
            result.final_distance = distance_traveled
            return result

        self.execute_stop()
        goal_handle.succeed()   

        result = MoveX.Result()
        result.success = True
        result.final_distance = distance_traveled
        self.get_logger().info(f'Target distance reached: {result.final_distance:.2f}m')
        return result

def main():
    rclpy.init()
    move_x_action_server = MoveXActionServer()
    rclpy.spin(move_x_action_server)
    move_x_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
