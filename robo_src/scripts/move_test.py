#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897

def rotate():
    #Starts a new node
    rospy.init_node('memes', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    msg = Twist()
    # msg.linear.x = 2
    msg.angular.z = 0.5

    while True:
        pub.publish(msg)

if __name__ == '__main__':
    try:
        # Testing our function
        rotate()
    except rospy.ROSInterruptException:
        pass
