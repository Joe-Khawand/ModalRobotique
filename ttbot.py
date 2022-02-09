#!/usr/bin/env python3
import rospy
from sensor_msgs import msg
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

rospy.init_node('ligne', anonymous=True)
pub1=rospy.Publisher('cmd_vel', Twist,queue_size=10 )

vel= Twist()
vel.linear.x=0.22

def dist(msg):
    if msg.ranges[0]> 0.5 :
        pub1.publish(vel)
    else:
        vel.linear.x=0
        pub1.publish(vel)

subs = rospy.Subscriber('/scan', LaserScan, dist)
    
rospy.spin()