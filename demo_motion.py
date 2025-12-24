#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class DemoJointPublisher(Node):
    def __init__(self):
        super().__init__('demo_joint_publisher')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.tick)

        suffixes = ['_r1', '_r2', '_r3', '_l1', '_l2', '_l3']
        names = ['coxa_joint', 'femur_joint', 'tibia_joint']
        self.joint_names = [f"{name}{suf}" for name in names for suf in suffixes]

    def tick(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = self.joint_names

        positions = []
        for idx, _name in enumerate(self.joint_names):
            phase = (idx % 6) * 0.4
            positions.append(0.4 * math.sin(2.0 * math.pi * 0.2 * t + phase))
        msg.position = positions

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = DemoJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
