
import rclpy
import math
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from maze_interfaces.action import MoveYaw


class MoveYawActionServer(Node):

    def _init_(self):
        super()._init_('move_yaw_action_server')
        
        self.callback_group = ReentrantCallbackGroup()
        
        self._action_server = ActionServer(
            self,
            MoveYaw,
            'move_yaw',
            self.execute_callback,
            callback_group=self.callback_group
        )
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.current_yaw = 0.0
        self.get_logger().info('MoveYaw Action Server initialized.')

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def stop_robot(self):
        twist = Twist()
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal: Turning...')
        
        target_angle = goal_handle.request.target_angle
        angular_speed = goal_handle.request.angular_speed if goal_handle.request.angular_speed != 0.0 else 0.5
        
        start_yaw = self.current_yaw
        feedback_msg = MoveYaw.Feedback()
        
        twist = Twist()
        twist.angular.z = angular_speed

        while rclpy.ok():
            angle_turned = abs(self.current_yaw - start_yaw)
            
            feedback_msg.current_yaw = self.current_yaw
            goal_handle.publish_feedback(feedback_msg)

            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                self.get_logger().info('Goal canceled.')
                result = MoveYaw.Result()
                result.success = False
                result.final_angle = angle_turned
                return result

            if angle_turned >= target_angle:
                self.stop_robot()
                break

            self.publisher_.publish(twist)
            time.sleep(0.05)

        goal_handle.succeed()
        
        result = MoveYaw.Result()
        result.success = True
        result.final_angle = abs(self.current_yaw - start_yaw)
        self.get_logger().info(f'Target angle reached: {result.final_angle:.2f} rad')
        return result


def main(args=None):
    rclpy.init(args=args)
    move_yaw_action_server = MoveYawActionServer()
    
    executor = MultiThreadedExecutor()
    executor.add_node(move_yaw_action_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        move_yaw_action_server.stop_robot()
        move_yaw_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '_main_':
    main()