#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node

from crab_msgs.msg import BodyCommand, GaitCommand


class DemoGait(Node):
    def __init__(self):
        super().__init__('demo_gait')
        self.body_cmd_pub = self.create_publisher(BodyCommand, '/teleop/body_command', 1)
        self.gait_pub = self.create_publisher(GaitCommand, '/teleop/gait_control', 1)

        self.stand_sent = False
        self.timer = self.create_timer(0.5, self.tick)
        self.start_time = time.time()

    def tick(self):
        now = time.time()
        if not self.stand_sent and now - self.start_time > 1.0:
            cmd = BodyCommand()
            cmd.cmd = BodyCommand.STAND_UP_CMD
            self.body_cmd_pub.publish(cmd)
            self.get_logger().info('Sent STAND_UP_CMD')
            self.stand_sent = True

        gait = GaitCommand()
        gait.cmd = GaitCommand.RUNTRIPOD
        gait.fi = 0.0
        gait.velocity = 0.05
        gait.alpha = 0.0
        gait.scale = 1.0
        self.gait_pub.publish(gait)


def main():
    rclpy.init()
    node = DemoGait()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
