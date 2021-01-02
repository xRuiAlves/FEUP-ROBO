import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

class PipelinedRetransmitter(Node):
    def __init__(self, pipeline, topic_from='scan', topic_to='scan_fi'):
        super().__init__('fault_injector')

        self.pipeline = pipeline

        self.create_subscription(
                LaserScan,
                topic_from,
                lambda sensor_data: self.sensor_callback(sensor_data),
                qos_profile=qos_profile_sensor_data
            )

        self.pub = self.create_publisher(
                LaserScan,
                topic_to,
                qos_profile_sensor_data
            )

    def sensor_callback(self, sensor_data):
        """
        Receives the sensor data message from a topic and mutates it according to the fault injection pipeline. Then, publishes the message.
        """
        msg = self.pipeline.mutate_message(sensor_data)
        self.pub.publish(msg)
