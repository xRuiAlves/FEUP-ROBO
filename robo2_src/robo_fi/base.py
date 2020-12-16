#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

import math
import random

class Base(Node):
    def __init__(self):
        super().__init__('fault_injector')
        qos = QoSProfile(depth=10)

        self.pub = self.create_publisher(
                LaserScan,
                'scan_fake',
                # 'scan',
                qos_profile_sensor_data
            )
        self.create_subscription(
                LaserScan,
                'scan',
                # 'scan_in',
                self.scan_callback,
                qos_profile=qos_profile_sensor_data
            )

    def scan_callback(self, sensor_data):
        # if sensor_data.ranges[0] == 1.0:
        #     return
        # print(sensor_data)
        msg = LaserScan()
        # copied
        msg.angle_min = math.radians(0)
        msg.angle_max = math.radians(360)
        msg.angle_increment = math.radians(1)
        msg.range_min = sensor_data.range_min
        msg.range_max = sensor_data.range_max
        msg.header = sensor_data.header
        # msg.ranges = sensor_data.ranges
        # spoofed
        msg.ranges = [0.0]*360
        for idx, x in enumerate(sensor_data.ranges):
            msg.ranges[idx] = 0.5 * x

        # for i in range(360):
        #     msg.ranges[i] = (i * (msg.range_max - msg.range_min) / 360) + msg.range_min
        # msg.ranges[0] = 1.0

        msg.intensities = [0.0]*360
        # print(msg)
        self.pub.publish(msg)

def main(args=None):
    print('Hi from robo_fi.')
    rclpy.init(args=args)
    base = Base()
    rclpy.spin(base)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    base.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

