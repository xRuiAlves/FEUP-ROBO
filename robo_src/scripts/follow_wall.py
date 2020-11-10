#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from statistics import mean

# constants
TARGET_WALL_DISTANCE = 0.16
ROBOT_SPEED = 0.05
P_WEIGHT = 3
D_WEIGHT = 1
# D_MIN = -1
# D_MAX = 1
NUM_REGRESSION_POINTS = 15

def moveRobot(pub, rotation):
    msg = Twist()
    msg.linear.x = ROBOT_SPEED
    msg.angular.z = rotation
    pub.publish(msg)

def getClosestAngle(sensor_data):
    closest_angle = None

    for angle in range(len(sensor_data.ranges)):
        if (isDistanceValid(sensor_data, sensor_data.ranges[angle])):
            if (closest_angle == None):
                closest_angle = angle
            elif (sensor_data.ranges[angle] < sensor_data.ranges[closest_angle]):
                closest_angle = angle

    return closest_angle

def getClosestPoints(sensor_data):
    closest_angle = getClosestAngle(sensor_data)
    if (closest_angle == None):
        return []

    distances = ((angle, sensor_data.ranges[angle]) for angle in range(len(sensor_data.ranges)))
    valid_distances = list(filter(lambda ang_dist: isDistanceValid(sensor_data, ang_dist[1]), distances))
    closest_distances = sorted(valid_distances, key=lambda ang_dist: ang_dist[1])[:NUM_REGRESSION_POINTS]
    return list(map(lambda ang_dist: getPoint(ang_dist[0], ang_dist[1]), closest_distances))

def getPoint(angle, distance):
    x = distance * math.cos(degreeToRad(angle))
    y = distance * math.sin(degreeToRad(angle))
    return (x, y)

def degreeToRad(ang_degrees):
    return (ang_degrees / 180) * math.pi

def isDistanceValid(sensor_data, distance):
    return distance != None and distance >= sensor_data.range_min and distance < sensor_data.range_max

def findLinearRegressionSlope(points):
    points_x = list(map(lambda point: point[0], points))
    points_y = list(map(lambda point: point[1], points))

    xs = np.array(points_x, dtype=np.float64)
    ys = np.array(points_y, dtype=np.float64)

    return (((mean(xs)*mean(ys)) - mean(xs*ys)) / ((mean(xs)**2) - mean(xs**2)))

def calcP(sensor_data):
    closest_angle = getClosestAngle(sensor_data)
    if (closest_angle == None):
        return 0

    rospy.loginfo(f"ang : {closest_angle}")
    distance = sensor_data.ranges[closest_angle]
    rospy.loginfo(f"dist: {distance}")
    diff = distance - TARGET_WALL_DISTANCE

    turning_sign = 1 if (closest_angle >= 0 and closest_angle <= 180) else -1
    return diff * turning_sign

def calcDist(point):
    return math.sqrt(point[0]**2 + point[1]**2)

def calcD(sensor_data):
    points = getClosestPoints(sensor_data)
    slope = findLinearRegressionSlope(points)
    return slope

def callback(sensor_data, pub):
    P = calcP(sensor_data) * P_WEIGHT
    D = calcD(sensor_data) * D_WEIGHT

    rospy.loginfo(f"P: {P}")
    rospy.loginfo(f"D: {D}")
    rospy.loginfo(f"---------------------")

    rotation = P + D
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
