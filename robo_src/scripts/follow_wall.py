#!/usr/bin/env python
import rospy
import math
import random
import sys
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from statistics import mean

# robot dicision sensors
SENSORS = {
    "FRONT": 0,
    "FRONTAL_LEFT": 45,
    "FRONTAL_RIGHT": 315,
    "LEFT": 90,
    "RIGHT": 270,
    "BACK_LEFT": 105,
    "BACK_RIGHT": 255
}

# distances
TARGET_WALL_DISTANCE = 0.17
DISTANCE_THRESHOLD = 2 * TARGET_WALL_DISTANCE
OBJECT_DETECTION_THRESHOLD = 3.2

# robot forward motion
SPEED = 0.16
SPEED_MULTIPLER = (float(sys.argv[1]) if (len(sys.argv) >= 2) else 1.8)
ROBOT_MAX_SPEED = 0.5

# robot turning motion
TURNING_MULTIPLIER = (float(sys.argv[2]) if (len(sys.argv) >= 3) else 5/4)
TURNING_FACTOR = SPEED * TARGET_WALL_DISTANCE * TURNING_MULTIPLIER * math.pi
TURNING_SPEED = TURNING_FACTOR * 2 * math.pi
MAX_RANDOM_TURN = math.pi/4
MIN_RANDOM_TURN = -MAX_RANDOM_TURN

# wall detection weights
DISTACE_DIFF_WEIGHT = 3
LIN_REGRESSION_WEIGHT = 5
NUM_REGRESSION_POINTS = 25


def moveRobot(pub, speed, rotation):
    msg = Twist()
    msg.linear.x = speed
    msg.angular.z = rotation
    pub.publish(msg)

def getAngleDistance(sensor_data, angle):
    distance = sensor_data.ranges[angle]
    return distance if isDistanceValid(sensor_data, distance) else None

def getClosestAngle(sensor_data):
    closest_angle = None

    for angle in range(len(sensor_data.ranges)):
        if (isDistanceValid(sensor_data, sensor_data.ranges[angle])):
            if (closest_angle == None):
                closest_angle = angle
            elif (sensor_data.ranges[angle] < sensor_data.ranges[closest_angle]):
                closest_angle = angle

    return closest_angle

def calcClosestPoints(sensor_data):
    closest_angle = getClosestAngle(sensor_data)
    if (closest_angle == None):
        return []

    distances = ((angle, sensor_data.ranges[angle]) for angle in range(len(sensor_data.ranges)))
    valid_distances = list(filter(lambda ang_dist: isDistanceValid(sensor_data, ang_dist[1]), distances))
    closest_distances = sorted(valid_distances, key=lambda ang_dist: ang_dist[1])[:NUM_REGRESSION_POINTS]
    return list(map(lambda ang_dist: calcPoint(ang_dist[0], ang_dist[1]), closest_distances))

def calcPoint(angle, distance):
    x = distance * math.cos(degreeToRad(angle))
    y = distance * math.sin(degreeToRad(angle))
    return (x, y)

def degreeToRad(ang_degrees):
    return (ang_degrees / 180) * math.pi

def isDistanceValid(sensor_data, distance):
    return distance != None and distance > sensor_data.range_min and distance < sensor_data.range_max

def calcLinearRegressionSlope(points):
    points_x = list(map(lambda point: point[0], points))
    points_y = list(map(lambda point: point[1], points))

    xs = np.array(points_x, dtype=np.float64)
    ys = np.array(points_y, dtype=np.float64)

    return (((mean(xs)*mean(ys)) - mean(xs*ys)) / ((mean(xs)**2) - mean(xs**2)))

def calcDistanceDiff(sensor_data):
    closest_angle = getClosestAngle(sensor_data)
    if (closest_angle == None):
        return 0

    distance = sensor_data.ranges[closest_angle]
    diff = distance - TARGET_WALL_DISTANCE

    turning_sign = 1 if (closest_angle >= 0 and closest_angle <= 180) else -1
    return diff * TURNING_SPEED * turning_sign * 2 * math.pi

def calcLinRegresionSlope(sensor_data):
    points = calcClosestPoints(sensor_data)
    slope = calcLinearRegressionSlope(points)
    return slope

def getAngIntervalDistSum(sensor_data, ang_from, ang_to):
    distances = sensor_data.ranges[ang_from : ang_to]
    return sum(filter(lambda distance: isDistanceValid(sensor_data, distance), distances))

def getRandomTurningAngle():
    return random.uniform(MIN_RANDOM_TURN, MAX_RANDOM_TURN)

def calcSpeedRotation(sensor_data):
    f = getAngleDistance(sensor_data, SENSORS["FRONT"])
    fl = getAngleDistance(sensor_data, SENSORS["FRONTAL_LEFT"])
    fr = getAngleDistance(sensor_data, SENSORS["FRONTAL_RIGHT"])
    l = getAngleDistance(sensor_data, SENSORS["LEFT"])
    r = getAngleDistance(sensor_data, SENSORS["RIGHT"])
    bl = getAngleDistance(sensor_data, SENSORS["BACK_LEFT"])
    br = getAngleDistance(sensor_data, SENSORS["BACK_RIGHT"])

    has_frontal_diagonals = fl != None or fr != None
    frontal_collision_danger = f != None and f <= DISTANCE_THRESHOLD

    has_close_left_wall = (l != None and l < DISTANCE_THRESHOLD) or (bl != None and bl < DISTANCE_THRESHOLD)
    has_close_right_wall = (r != None and r < DISTANCE_THRESHOLD) or (br != None and br < DISTANCE_THRESHOLD)

    should_turn_left = has_close_left_wall and (fl == None or fl >= 1.15 * DISTANCE_THRESHOLD)
    should_turn_right = has_close_right_wall and (fr == None or fr >= 1.15 * DISTANCE_THRESHOLD)

    if (frontal_collision_danger):
        rospy.loginfo(">> FRONTAL COLLISION <<")
        should_turn_left = bl == None or (br != None and bl > br)
        return 0, 2.5 * TURNING_SPEED * (1 if should_turn_left else -1)
    if (should_turn_right and has_close_left_wall):
        rospy.loginfo(">> TURNING SOFT LEFT <<")
        return SPEED_MULTIPLER * SPEED, 0.8 * TURNING_SPEED
    if (should_turn_left and has_close_right_wall):
        rospy.loginfo(">> TURNING SOFT RIGHT <<")
        return SPEED_MULTIPLER * SPEED, 0.8 * TURNING_SPEED * -1
    if (should_turn_left):
        rospy.loginfo(">> TURNING LEFT <<")
        return SPEED_MULTIPLER * SPEED, 1.5 * TURNING_SPEED
    if (should_turn_right):
        rospy.loginfo(">> TURNING RIGHT <<")
        return SPEED_MULTIPLER * SPEED, 1.5 * TURNING_SPEED * -1
    if (has_close_left_wall or has_close_right_wall):
        rospy.loginfo(">> FOLLOWING WALL <<")
        distance_diff = calcDistanceDiff(sensor_data) * DISTACE_DIFF_WEIGHT
        lin_regression_slope = calcLinRegresionSlope(sensor_data) * LIN_REGRESSION_WEIGHT
        rotation = distance_diff + lin_regression_slope
        return (SPEED_MULTIPLER if rotation > 0 else (SPEED_MULTIPLER - 0.2)) * SPEED, rotation
    else:
        closest_angle = getClosestAngle(sensor_data)
        if (closest_angle == None):
            rospy.loginfo(">> WANDERING WITHOUT TARGET<<")
            return ROBOT_MAX_SPEED, getRandomTurningAngle()
        else:
            rospy.loginfo(">> WANDERING WITH TARGET<<")
            lin_regression_slope = calcLinRegresionSlope(sensor_data) * LIN_REGRESSION_WEIGHT
            angle_quadrant_sign = (-1 if (closest_angle > 90 and closest_angle < 270) else 1)
            angle_slope_sign = (1 if lin_regression_slope < 0 else -1)
            rotation = math.pi/4 * angle_slope_sign * angle_quadrant_sign
            return SPEED_MULTIPLER * SPEED, rotation

def callback(sensor_data, pub):
    speed, rotation = calcSpeedRotation(sensor_data)
    rospy.loginfo(f"Speed = {speed}")
    rospy.loginfo(f"Rotation = {rotation}")
    rospy.loginfo("--------------------")
    moveRobot(pub, speed, rotation)
    
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
