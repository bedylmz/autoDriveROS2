#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

class Drive(Node):
    def __init__(self):
        super().__init__('drive')
        self.navigator = BasicNavigator()
        self.send_goal()

    def send_goal(self):
        # Set initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = -1.9
        initial_pose.pose.position.y = -0.5
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()
        time.sleep(2.0)

        # Define goal pose
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = 1.9
        goal.pose.position.y = 0.5
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        # Send goal
        self.navigator.goToPose(goal)
        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled.')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')

def main(args=None):
    rclpy.init(args=args)
    node = Drive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
