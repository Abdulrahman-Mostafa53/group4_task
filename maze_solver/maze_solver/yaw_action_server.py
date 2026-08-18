import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from my_robot_interfaces.action import MoveYaw
from my_robot_interfaces.srv import Error


# Define and initialize the MoveYaw Action Server node class
class MoveYawActionServer(Node):

    # Constructor method to initialize node, ROS 2 communication interfaces, and state variables
    def __init__(self):
        # Initialize the node with the name 'move_yaw_action_server'
        super().__init__('move_yaw_action_server')

        # Create the Action Server for handling rotation movement goals based on direction
        self._action_server = ActionServer(
            self,
            MoveYaw,
            'move_yaw',
            self.execute_callback
        )

        # Create Publisher to send angular velocity commands to the robot
        self.publisher_ = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )

        # Create Subscriber to continuously monitor robot odometry and orientation
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Variables to track the robot's current orientation and execution state
        self.current_yaw = 0.0
        self.is_executing_goal = False   # Tracks if a rotation goal is currently running
        self.start_yaw_goal = 0.0        # Records the starting angle when the rotation began
        self.current_direction = ""      # Records whether it was moving 'left' or 'right'
        
        # Create Service Server to handle manual stop/emergency requests with automatic return logic
        self.srv = self.create_service(
            Error,
            'stop_robot',
            self.stop_robot_callback
        )
        
        # Log node startup and readiness status
        self.get_logger().info('MoveYaw Action Server & Stop Robot Service initialized.')

    # Helper function to stop the robot rotation immediately with zero angular velocity
    def execute_stop(self):
        # Prepare a zero-angular-velocity Twist message to halt rotation safely
        zero_angular_velocity = Twist()
        zero_angular_velocity.angular.z = 0.0
        
        # Publish zero angular velocity to stop the robot rotation
        self.publisher_.publish(zero_angular_velocity)
        # ملاحظة: تم إزالة تصفير current_yaw هنا عشان ميبوظش قراءات الـ Odometry الحقيقية

    # Helper function to perform recovery return to start orientation when interrupted
    def perform_recovery_return(self, start_yaw, direction):
        self.get_logger().info('Executing recovery: Returning to original starting orientation...')
        
        # Determine reverse angular speed based on the direction that was being executed
        base_speed = 1.0
        
        recovery_twist = Twist()
        
        while rclpy.ok():
            current_diff = self.normalize_angle(self.current_yaw - start_yaw)
            # If we are close enough to the original starting orientation, stop loop
            if abs(current_diff) < 0.02:
                break
            
            # Apply dynamic recovery angular velocity command (opposite to original direction)
            recovery_twist.angular.z = -base_speed if current_diff > 0 else base_speed
            self.publisher_.publish(recovery_twist)
            time.sleep(0.05)

        # Stop rotation completely with zero angular velocity after reaching origin
        self.execute_stop()
        self.get_logger().info('Successfully returned to original orientation via stop service.')

    # Service callback function to process incoming emergency stop requests and trigger return logic
    def stop_robot_callback(self, request, response):
        # Check if the incoming request flag is set to True
        if request.stop:
            # Check if the robot is currently executing an action goal
            if self.is_executing_goal:
                self.get_logger().info('Stop service triggered during active movement. Forcing recovery return...')
                # Perform the automatic return to the starting position/orientation
                self.perform_recovery_return(self.start_yaw_goal, self.current_direction)
                
                # Reset execution flags
                self.is_executing_goal = False
                
                response.success = True
                response.message = 'Robot stopped and successfully returned to original orientation.'
            else:
                # If no goal is active, apply zero angular velocity to keep it idle
                self.execute_stop()
                
                response.success = True
                response.message = 'Robot is idle. Zero angular velocity applied.'
                
            self.get_logger().info('Stop service executed successfully!')
        else:
            self.get_logger().info('Received stop request as False, doing nothing.')
            response.success = False
            response.message = 'Stop request was False, no action taken.'

        return response

    # Odometry callback function to continuously extract and update current Yaw angle from quaternion
    def odom_callback(self, msg):
        # Extract orientation quaternion components from odometry message
        orientation_q = msg.pose.pose.orientation
        qx = orientation_q.x
        qy = orientation_q.y
        qz = orientation_q.z
        qw = orientation_q.w

        # Convert quaternion to Euler angles (specifically Yaw) using mathematical formula
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    # Helper function to safely normalize angle differences within the range [-pi, pi]
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # Main action server callback function to execute rotation towards a fixed 90-degree target based on direction
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal: Rotating robot...')
        
        # Extract target direction from goal request ('left' or 'right')
        direction = goal_handle.request.direction.lower()
        
        # Validate direction input
        if direction not in ['left', 'right']:
            self.get_logger().error(f"Invalid direction received: '{direction}'. Expected 'left' or 'right'.")
            goal_handle.abort()
            result = MoveYaw.Result()
            result.success = False
            result.final_direction = "none"
            return result

        # Mark execution state variables
        self.is_executing_goal = True
        self.current_direction = direction
        self.start_yaw_goal = self.current_yaw

        # Fixed target angle value in radians (90 degrees = pi / 2 radians)
        target_angle_rad = math.pi / 2.0
        
        # Set angular rotation speed locally
        base_speed = 1.0
        if direction == 'left':
            angular_speed = base_speed
            target_sign = 1.0
        else:
            angular_speed = -base_speed
            target_sign = -1.0

        # Initialize feedback message container
        feedback_msg = MoveYaw.Feedback()

        # Prepare angular velocity message for rotation
        angular_twist = Twist()
        angular_twist.angular.z = angular_speed

        # Initialize angle traveled variable before loop
        angle_traveled = 0.0

        # Control loop: Keep rotating while node is active, goal is active, and target angle is not reached
        while rclpy.ok() and self.is_executing_goal and (abs(angle_traveled) < target_angle_rad):
            # Calculate total angle rotated so far relative to the starting yaw
            yaw_diff = self.current_yaw - self.start_yaw_goal
            angle_traveled = self.normalize_angle(yaw_diff)

            # Publish current feedback (current yaw angle) to client
            feedback_msg.current_yaw = self.current_yaw
            goal_handle.publish_feedback(feedback_msg)

            # Check if action cancellation was requested by client action interface
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled via action client! Returning robot...')
                self.perform_recovery_return(self.start_yaw_goal, self.current_direction)
                self.is_executing_goal = False

                result = MoveYaw.Result()
                result.success = False
                result.final_direction = "canceled_" + direction
                return result

            # Publish rotational angular velocity command to robot
            angular_twist.angular.z = angular_speed
            self.publisher_.publish(angular_twist)
            
            # Short sleep delay to control publication loop rate
            time.sleep(0.05)

        # If execution was interrupted externally by service, handle gracefully
        if not self.is_executing_goal:
            result = MoveYaw.Result()
            result.success = False
            result.final_direction = "stopped_via_service_" + direction
            goal_handle.abort()
            return result

        # Stop robot rotation with zero angular velocity after reaching target 90-degree angle
        self.execute_stop()
        
        # Reset execution flag upon successful completion
        self.is_executing_goal = False
        
        # Mark goal execution as successful
        goal_handle.succeed()   

        # Prepare and return final result with the successfully executed direction
        result = MoveYaw.Result()
        result.success = True
        result.final_direction = direction  
        self.get_logger().info(f'Target 90-degree rotation reached successfully. Final direction: {result.final_direction}')
        return result


# Main function to initialize ROS 2 communication, create node, and spin execution
def main():
    # Initialize ROS 2 communication
    rclpy.init()
    
    # Instantiate the node
    move_yaw_action_server = MoveYawActionServer()
    
    # Keep node alive to process callbacks and requests
    rclpy.spin(move_yaw_action_server)
    
    # Destroy node explicitly when spinning stops
    move_yaw_action_server.destroy_node()
    
    # Shutdown ROS 2 communication
    rclpy.shutdown()


# Entry point execution guard
if __name__ == '__main__':
    main()
