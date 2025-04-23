#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import time

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(Pose, '/target_pose', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        pose = Pose()
        pose.position.x = 0.3
        pose.position.y = 0.0
        pose.position.z = 0.2
        pose.orientation.w = 1.0  # 정방향

        self.publisher_.publish(pose)
        self.get_logger().info('✅ Published target pose to /target_pose')

        self.count += 1
        if self.count >= 3:  # 3번 발행 후 종료
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

