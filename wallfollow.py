#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from math import*

#PID CONTROL PARAMS
kp = 3.2#TODO
kd = 0#TODO
ki = 0#TODO
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.6 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 3 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        #drive_topic = '/vesc/low_level/ackermann_cmd_mux/input/navigation'
        drive_topic2= '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic,LaserScan,self.lidar_callback) #TODO: Subscribe to LIDAR
        self.drive_pub = rospy.Publisher(drive_topic2,AckermannDriveStamped,queue_size=50) #TODO: Publish to drive

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        a= data.angle_min
        inc=data.angle_increment
        i= int((angle-np.pi/2-a)/inc)
        if( not math.isnan(data.ranges[i])) and data.ranges[i] <= data.range_max:
            return data.ranges[i]
        else:
            return data.range_max
        
    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        
        integral += error
        angle = kp*error+ki*(integral)+kd*(error-prev_error)
        
        #TODO: Use kp, ki & kd to implement a PID controller for 
        if abs(angle*180/np.pi)<10 : velocity=3
        elif abs(angle*180/np.pi)< 20 : velocity=2
        else : velocity=1
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
        prev_error=error

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 
        #TODO:implement
        return 0.0 

    def lidar_callback(self, data):
        a=self.getRange(data,(np.pi)/3)
        b=self.getRange(data,0)
        
        ang=np.pi/3
        alpha=np.arctan2((a*np.cos(ang)-b),(a*np.sin(ang)))
        
        error = DESIRED_DISTANCE_RIGHT - (b*np.cos(alpha)+CAR_LENGTH*np.sin(alpha))
        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)