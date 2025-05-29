#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def send_goal(navigator, x: float, y: float, frame: str = 'map'):
    goal = PoseStamped()
    goal.header.frame_id = frame
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.w = 1.0

    navigator.goToPose(goal)

    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.1)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f"Reached ({x:.2f}, {y:.2f})")
    elif result == TaskResult.CANCELED:
        print(f"Canceled reaching ({x:.2f}, {y:.2f})")
    else:
        print(f"Failed to reach ({x:.2f}, {y:.2f})")

def main():
    rclpy.init()
    navigator = BasicNavigator()
    node = navigator  

    def odom_cb(msg: Odometry):
        node.latest_odom = msg
    node.create_subscription(Odometry, '/odom', odom_cb, 10)

    while not hasattr(node, 'latest_odom'):
        rclpy.spin_once(node, timeout_sec=0.1)

    odom = node.latest_odom.pose.pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = node.get_clock().now().to_msg()
    initial_pose.pose.position.x = odom.position.x
    initial_pose.pose.position.y = odom.position.y
    initial_pose.pose.position.z = odom.position.z
    initial_pose.pose.orientation.x = odom.orientation.x
    initial_pose.pose.orientation.y = odom.orientation.y
    initial_pose.pose.orientation.z = odom.orientation.z
    initial_pose.pose.orientation.w = odom.orientation.w

    navigator.setInitialPose(initial_pose)




    navigator.waitUntilNav2Active()

 
    waypoints = [
        (5.0,   5.0),   
        (10.0,  0.0),   
        (0.0,  10.0),  
        (-10.0, 0.0),  
        (0.0,  -10.0), 
    ]
    for x, y in waypoints:
        send_goal(navigator, x, y)

  
    navigator.lifecycleShutdown()
    navigator.destroyNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
