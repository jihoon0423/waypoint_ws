#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
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
    navigator.waitUntilNav2Active()

    waypoints = [
        (5.0,   5.0),   # 맵의 북동쪽 코너 근처
        (10.0,  0.0),   # 맵의 동쪽 중앙
        (0.0,  10.0),   # 맵의 북쪽 중앙
        (-10.0, 0.0),   # 맵의 서쪽 중앙
        (0.0,  -10.0),  # 맵의 남쪽 중앙
    ]
    for x, y in waypoints:
        send_goal(navigator, x, y)

    navigator.lifecycleShutdown()
    navigator.destroyNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
