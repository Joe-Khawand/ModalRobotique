#!/usr/bin/env python3
import rospy
import numpy as np
import atexit
import tf
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

home = expanduser('~')
file = open(home+'/waypoint.csv', 'w') 

def save_waypoint(data):
    file.write('%f, %f\n' % (data.pose.position.x,
                            data.pose.position.y))         
def shutdown():
    file.close()
    print('Goodbye')
 
def listener():
    rospy.init_node('waypoints_logger', anonymous=True)
    rospy.Subscriber('/gt_pose', PoseStamped, save_waypoint)
    rospy.spin()

if __name__ == '__main__':
    atexit.register(shutdown)
    print('Saving waypoints...')
    try:
        listener()
    except rospy.ROSInterruptException:
        pass