#!/usr/bin/env python
import rospy
import math
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

# constants
TARGET_WALL_DISTANCE = 0.17
ROBOT_SPEED = 0.15

DISTANCE_THRESHOLD = 2 * TARGET_WALL_DISTANCE
DISTACE_DIFF_WEIGHT = 3
LIN_REGRESSION_WEIGHT = 5
NUM_REGRESSION_POINTS = 25

TURNING_FACTOR = ROBOT_SPEED * TARGET_WALL_DISTANCE * 5/4 * math.pi
TURNING_SPEED = TURNING_FACTOR * 2 * math.pi

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

def calcSpeedRotation(sensor_data):
    f = getAngleDistance(sensor_data, SENSORS["FRONT"])
    fl = getAngleDistance(sensor_data, SENSORS["FRONTAL_LEFT"])
    fr = getAngleDistance(sensor_data, SENSORS["FRONTAL_RIGHT"])
    l = getAngleDistance(sensor_data, SENSORS["LEFT"])
    r = getAngleDistance(sensor_data, SENSORS["RIGHT"])

    has_frontal_diagonals = fl != None or fr != None
    frontal_collision_danger = f != None and f <= DISTANCE_THRESHOLD

    has_close_left_wall = (l != None and l < DISTANCE_THRESHOLD)
    has_close_right_wall = (r != None and r < DISTANCE_THRESHOLD)
    should_turn_left = has_close_left_wall and (fl == None or fl >= DISTANCE_THRESHOLD)
    should_turn_right = has_close_right_wall and (fr == None or fr >= DISTANCE_THRESHOLD)

    if (frontal_collision_danger):
        rospy.loginfo(f"FRONTAL_COLLISION")
        bl = getAngleDistance(sensor_data, SENSORS["BACK_LEFT"])
        br = getAngleDistance(sensor_data, SENSORS["BACK_RIGHT"])
        should_turn_left = bl == None or (br != None and bl > br)
        return 0, 1.5 * TURNING_SPEED * (1 if should_turn_left else -1)
    if (should_turn_left):
        rospy.loginfo(f"TURNING LEFT")
        return ROBOT_SPEED, TURNING_SPEED
    if (should_turn_right):
        rospy.loginfo(f"TURNING RIGHT")
        return ROBOT_SPEED, -TURNING_SPEED
    if (has_close_left_wall or has_close_right_wall):
        rospy.loginfo(f"FOLLOWING WALL")
        distance_diff = calcDistanceDiff(sensor_data) * DISTACE_DIFF_WEIGHT
        lin_regression_slope = calcLinRegresionSlope(sensor_data) * LIN_REGRESSION_WEIGHT
        return 1.5 * ROBOT_SPEED, distance_diff + lin_regression_slope
    else:
        rospy.loginfo(f"FAK\n\tl = {l}\n\tr = {r}\n\tfl = {fl}\n\tfr = {fr}")
        return ROBOT_SPEED, 0

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
