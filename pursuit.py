#!/usr/bin/env python3
from cmath import sqrt
from turtle import distance
import rospy
import csv
import ackermann_msgs
import math as m
import numpy as np
import tf
from tf.transformations import euler_from_quaternion
from os.path import expanduser
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Bool

# TODO: import ROS msg types and libraries


home = expanduser('~')
file = open(home+'/waypoint.csv')
csvreader = csv.reader(file)
points = []
inter=0
kp=1

#creation d'un tableau [x,y,x,y,x,y .....]
for row in csvreader:
        for i in row:
            points.append(float(i))


file.close()
print(len(points))
velocity=1
p=1000
L=1
xp= float(points[0])
yp= float(points[1])

def distance(a,b,c,d):
    
    return m.sqrt((c-a)**2+(d-b)**2)

class PurePursuit(object):
    
    #The class that handles pure pursuit.
    
    def __init__(self):
        # TODO: create ROS subscribers and publishers.
        pos_sub = rospy.Subscriber('/gt_pose', PoseStamped, self.pose_callback)
        self.drive_pub = rospy.Publisher('/nav',AckermannDriveStamped,queue_size=50)

      

    def pose_callback(self, pose_msg):
        global p
        global points
        global L
        global xp
        global yp
        global kp

        xr= float(pose_msg.pose.position.x)
        yr= float(pose_msg.pose.position.y)

        # TODO: find the current waypoint to track using methods mentioned in lecture
        l= distance(xr,yr,points[p+2],points[p+3])

        if l<= L: #nouveau point dans le cercle
             p=p+100
             xp=points[p]
             yp=points[p+1]

        else: #2 options interpoler ou continuer a aller vers le point(rien faire)
            if distance(xr,yr,xp,yp)<L/4:#interpole
                xp=xr+(points[p+200]-xr)*(L/l)
                yp=yp+(points[p+201]-yr)*(L/l)

        # TODO: calculate curvature/steering angle

        quat = pose_msg.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        yaw = m.atan2(siny_cosp, cosy_cosp)
        tetapoint = m.atan2(yp - yr, xp - xr)

        alpha = tetapoint - yaw 

        while alpha > np.pi:
            alpha -= 2 * np.pi
        while alpha <= - np.pi:
            alpha += 2 * np.pi

        if alpha > np.pi / 2:
            curv = 0.4188
        elif alpha < - np.pi / 2:
            curv = - 0.4188
        else:
            y = l * m.sin(alpha) 
            curv = 2 * y / (l*l)
            while curv > np.pi:
                curv -= 2 * np.pi
            
            while curv <= - np.pi:
                curv += 2 * np.pi
            
            curv = min(curv, 0.4188)
            curv = max(curv, -0.4188)

        # Publish drive message
        drive_msg=AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "pursuit"
        drive_msg.drive.steering_angle = curv
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()

if __name__ == '__main__':
    main()


