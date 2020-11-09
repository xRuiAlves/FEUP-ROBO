#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# constants
TARGET_WALL_DISTANCE = 1
ROBOT_SPEED = TARGET_WALL_DISTANCE / 2

TARGET_DIAGONAL_DISTANCE = TARGET_WALL_DISTANCE / math.sin(math.pi / 4)

TURNING_FACTOR = 0.65
TURNING_SPEED = (ROBOT_SPEED * TURNING_FACTOR) / TARGET_WALL_DISTANCE

# sensors
SENSORS = {
    "SENSOR_FL": 45,     # front-left
    "SENSOR_L": 90,      # left
    "SENSOR_R": 270,     # right
    "SENSOR_FR": 315     # front-right
}

def moveRobot(pub, rotation):
    msg = Twist()
    msg.linear.x = ROBOT_SPEED
    msg.angular.z = rotation
    pub.publish(msg)

def getSensorDistance(sensor_data, angle):
    distance = sensor_data.ranges[angle]
    return distance if isDistanceValid(sensor_data, distance) else None
        
def getSensorDistances(sensor_data):
    return list(map(
        lambda angle: getSensorDistance(sensor_data, angle),
        SENSORS.values()
    ))

def isDistanceValid(sensor_data, distance):
    return distance >= sensor_data.range_min and distance <= sensor_data.range_max

def callback(sensor_data, pub):
    [fl, l, r, fr] = getSensorDistances(sensor_data)

    should_follow_wall = (fl != None) or (fr != None)
    should_turn = (not should_follow_wall) and ((l != None) or (r != None))

    rotation = 0

    if (should_follow_wall):
        rospy.loginfo("Following wall")
        if (fl != None):
            rospy.loginfo(f"\tfront_left = {fl}")
            rotation += fl - TARGET_DIAGONAL_DISTANCE
        if (fr != None):
            rospy.loginfo(f"\tfront_right = {fr}")
            rotation -= fr - TARGET_DIAGONAL_DISTANCE
    elif (should_turn):
        rospy.loginfo("Turning")
        if (l != None):
            rotation = TURNING_SPEED
            rospy.loginfo("Turning left")
        elif (r != None):
            rotation = -TURNING_SPEED
            rospy.loginfo("Turning right")

    moveRobot(pub, rotation)
    
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
