#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# constants
TURNING_FACTOR = 1
ROBOT_SPEED = 0.25
TARGET_WALL_DISTANCE = 1
TARGET_DIAGONAL_DISTANCE = math.sqrt(math.pow(TARGET_WALL_DISTANCE, 2) + math.pow(TARGET_WALL_DISTANCE, 2))

# sensors
SENSORS = {
    "SENSOR_FL": 45,     # front-left
    # "SENSOR_BL": 135,    # back-left
    # "SENSOR_BR": 225,    # back-right
    "SENSOR_FR": 315     # front-right
}

def moveRobot(pub, spin):
    msg = Twist()
    msg.linear.x = ROBOT_SPEED
    msg.angular.z = spin
    pub.publish(msg)

def getSensorDistance(sensor_data, angle):
    return sensor_data.ranges[angle]

def getSensorDistances(sensor_data):
    return list(map(
        lambda angle: getSensorDistance(sensor_data, angle),
        SENSORS.values()
    ))

def isMeasureValid(sensor_data, measure):
    return measure >= sensor_data.range_min and measure <= sensor_data.range_max

def callback(sensor_data, pub):
    [fl, fr] = getSensorDistances(sensor_data)
    spin = 0

    if (isMeasureValid(sensor_data, fl)):
        spin += fl - TARGET_DIAGONAL_DISTANCE
    if (isMeasureValid(sensor_data, fr)):
        spin += TARGET_DIAGONAL_DISTANCE - fr

    rospy.loginfo([fl, fr])
    moveRobot(pub, spin)

    
def listener():
    rospy.init_node('robot', anonymous=True)

    sub = rospy.Subscriber('/scan', LaserScan, lambda sensor_data: callback(sensor_data, pub))
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
