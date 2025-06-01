#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import json
from datetime import datetime

PHOTO_DIR = os.path.expanduser('~/waypoint_photos')

class WaypointNavigator(BasicNavigator):
    def __init__(self):
        super().__init__()


        self.br = CvBridge()

        self.latest_image = None
        self.latest_pose = None

        self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            10
        )


        self.create_subscription(
            Odometry,
            '/odom',  
            self.pose_callback,
            10
        )

    def image_callback(self, msg: Image):
        self.latest_image = msg

    def pose_callback(self, msg: Odometry):
        self.latest_pose = msg.pose.pose

    def take_photo(self, waypoint_idx: int):
        timeout = self.get_clock().now().nanoseconds + 5_000_000_000
        while rclpy.ok() and (self.latest_image is None) and \
              (self.get_clock().now().nanoseconds < timeout):
            rclpy.spin_once(self, timeout_sec=0.1)


        now_msg = self.get_clock().now().to_msg()

        now_sec = now_msg.sec + now_msg.nanosec * 1e-9
        now_dt = datetime.utcfromtimestamp(now_sec)
        timestamp_iso = now_dt.isoformat() + f"{now_msg.nanosec:09d}"

        metadata = {
            'waypoint_id': waypoint_idx + 1,
            'timestamp': timestamp_iso,
            'pose': None,            
            'success': False,        
            'image_filename': None   
        }


        if self.latest_pose is not None:
            metadata['pose'] = {
                'position': {
                    'x': self.latest_pose.position.x,
                    'y': self.latest_pose.position.y,
                    'z': self.latest_pose.position.z
                },
                'orientation': {
                    'x': self.latest_pose.orientation.x,
                    'y': self.latest_pose.orientation.y,
                    'z': self.latest_pose.orientation.z,
                    'w': self.latest_pose.orientation.w
                }
            }
        else:
            metadata['pose'] = {
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
            }


        os.makedirs(PHOTO_DIR, exist_ok=True)

        if self.latest_image is None:

            self.get_logger().warning(f'Waypoint {waypoint_idx+1}: No camera image received, marking as failure.')


            json_filename = os.path.join(PHOTO_DIR, f'waypoint_{waypoint_idx + 1}.json')
            with open(json_filename, 'w') as jf:
                json.dump(metadata, jf, indent=4, ensure_ascii=False)


            self.latest_image = None
            return


        cv_image = self.br.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        image_filename = os.path.join(PHOTO_DIR, f'waypoint_{waypoint_idx + 1}.png')
        cv2.imwrite(image_filename, cv_image)
        self.get_logger().info(f'Waypoint {waypoint_idx+1}: Saved photo to: {image_filename}')

        metadata['success'] = True
        metadata['image_filename'] = image_filename


        json_filename = os.path.join(PHOTO_DIR, f'waypoint_{waypoint_idx + 1}.json')
        with open(json_filename, 'w') as jf:
            json.dump(metadata, jf, indent=4, ensure_ascii=False)


        self.latest_image = None


def send_goal(navigator: WaypointNavigator, goal: PoseStamped, idx: int):
    goal.header.stamp = navigator.get_clock().now().to_msg()
    navigator.goToPose(goal)


    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.1)

    result = navigator.getResult()
    x = goal.pose.position.x
    y = goal.pose.position.y

    if result == TaskResult.SUCCEEDED:
        print(f"[Waypoint {idx+1}] Reached ({x:.2f}, {y:.2f})")
        navigator.take_photo(idx)
    else:
        print(f"[Waypoint {idx+1}] Failed to reach ({x:.2f}, {y:.2f})")
        navigator.take_photo(idx)


def main():
    rclpy.init()
    navigator = WaypointNavigator()


    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -0.3527659295151565
    initial_pose.pose.position.y = -3.292957691502578e-11
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = 0.9981621357977354
    initial_pose.pose.orientation.w = -0.06059992293479631

    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()


    waypoints = []

    wp1 = PoseStamped()
    wp1.header.frame_id = 'map'
    wp1.pose.position.x =  4.574370434309949
    wp1.pose.position.y =  5.194985176338236
    wp1.pose.position.z =  0.0
    wp1.pose.orientation.x = 0.0
    wp1.pose.orientation.y = 0.0
    wp1.pose.orientation.z = 0.1533114364715118
    wp1.pose.orientation.w = 0.9881779209469526
    waypoints.append(wp1)

    wp2 = PoseStamped()
    wp2.header.frame_id = 'map'
    wp2.pose.position.x =  2.8336787738753957
    wp2.pose.position.y = -1.7789931122781792
    wp2.pose.position.z =  0.0
    wp2.pose.orientation.x = 0.0
    wp2.pose.orientation.y = 0.0
    wp2.pose.orientation.z = -0.8518522017756794
    wp2.pose.orientation.w =  0.5237822317814219
    waypoints.append(wp2)

    wp3 = PoseStamped()
    wp3.header.frame_id = 'map'
    wp3.pose.position.x = -4.556723158604956
    wp3.pose.position.y = -1.0558692915088466
    wp3.pose.position.z =  0.0
    wp3.pose.orientation.x = 0.0
    wp3.pose.orientation.y = 0.0
    wp3.pose.orientation.z = -0.9280783915080745
    wp3.pose.orientation.w = -0.3723848804876282
    waypoints.append(wp3)

    for idx, wp in enumerate(waypoints):
        send_goal(navigator, wp, idx)

    print("All waypoints done. Returning to dock.")
    dock_pose = PoseStamped()
    dock_pose.header.frame_id = 'map'
    dock_pose.header.stamp = navigator.get_clock().now().to_msg()
    dock_pose.pose = initial_pose.pose

    navigator.goToPose(dock_pose)
    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.1)
    print("Arrived at dock.")

    navigator.lifecycleShutdown()
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
