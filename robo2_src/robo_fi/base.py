#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

import math
import random
import sys

class Base(Node):    
    def __init__(self):
        super().__init__('fault_injector')
        qos = QoSProfile(depth=10)

        injection_callbacks = {
            "field_scale": self.field_scale_injection,
            "set_field_fixed": self.set_field_fixed_injection,
            "set_field_random": self.set_field_random_injection,
            "set_field_null": self.set_field_null_injection,
            "send_nothing": self.send_nothing
        }

        if (len(sys.argv) < 2):
            print("Missing arguments!")
            print("usage: base <injection type> [<argument>]")
            sys.exit(1)
        
        injection = sys.argv[1]
        argument = (sys.argv[2] if len(sys.argv) >= 3 else None)
        if (injection not in injection_callbacks.keys()):
            print("Invalid injection type!")
            print("Available types:")
            for available_injection in injection_callbacks.keys():
                print(f"\t{available_injection}")
            sys.exit(2)

        print(f"Injection: {injection}")
        if argument != None:
            print(f"Argument: {argument}")
        else:
            argument = -1

        self.pub = self.create_publisher(
                LaserScan,
                'scan_fi',
                qos_profile_sensor_data
            )

        self.create_subscription(
                LaserScan,
                'scan',
                lambda sensor_data: injection_callbacks.get(injection)(sensor_data, float(argument)),
                qos_profile=qos_profile_sensor_data
            )

    def field_scale_injection(self, sensor_data, factor):
        msg = sensor_data
        msg.ranges = [(factor * val) for val in msg.ranges]
        self.pub.publish(msg)

    def set_field_fixed_injection(self, sensor_data, value):
        msg = sensor_data
        msg.ranges = [value] * len(msg.ranges)
        self.pub.publish(msg)

    def set_field_random_injection(self, sensor_data, _value):
        msg = sensor_data
        msg.ranges = [random.uniform(0, 10) for val in msg.ranges]
        self.pub.publish(msg)

    def set_field_null_injection(self, sensor_data, _value):
        msg = sensor_data
        msg.ranges = [None] * len(msg.ranges)
        self.pub.publish(msg)

    def send_nothing(self, _sensor_data, _value):
        pass

def main(args=None):
    print("Fault injection system initialized.\n")
    rclpy.init(args=args)
    base = Base()
    rclpy.spin(base)

    base.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

