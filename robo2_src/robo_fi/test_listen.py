#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

class TestListen(Node):
    def __init__(self):
        super().__init__('fault_injector_test_listener')

        self.create_subscription(
                LaserScan,
                'scan_fake',
                self.scan_callback,
                qos_profile=qos_profile_sensor_data
            )

    def scan_callback(self, sensor_data):
        self.get_logger().info('Got message')
        self.get_logger().info(repr(sensor_data))

def main():
    print('Hi from a listener tester.')
    rclpy.init()
    tl = TestListen()
    rclpy.spin(tl)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
