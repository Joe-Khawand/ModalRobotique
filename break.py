#!/usr/bin/env python3
from threading import BrokenBarrierError
import ackermann_msgs
import rospy
import math



from sensor_msgs import msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from math import cos


class Safety(object):
    """
    La classe qui s'occupe du freinage d'urgence
    """
    
    def __init__(self):
        """
        /brake topic: Topic pour envoyer les instructions au robot.(message de type AckermannDriveStamped)
        /brake_bool topic: Topic pour activer le freinage(message de type Bool).
        /scan topic: donnees du lidar
        /odom topic: permet de recuperer la vitesse a partir des donnees odometriques
        
        """
        #brake= "/vesc/low_level/ackermann_cmd_mux/input/safety"
        brake2='/brake'
        #odom="/vesc/odom"
        odom2="/odom"

        self.speed = 0
        self.break_pub= rospy.Publisher(brake2,AckermannDriveStamped, queue_size=50)
        self.breakbool_pub = rospy.Publisher("/brake_bool",Bool, queue_size=50)
        odom_sub = rospy.Subscriber(odom2, Odometry, self.odom_callback)
        scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        # Creation des ROS subscribers et publishers.

    def odom_callback(self, odom_msg):
        #  Mise a jour de la vitesse
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # Calcul du Temps de collision TTC
        # comme le robot se colle au mur j'ai decide de voir juste l'angle devant pour eviter des arret soudains
        #for i in range(0, len(scan_msg.ranges)):
            i=-abs(scan_msg.angle_min)
            k=0
            while i<0:
                i=i+scan_msg.angle_increment
                k+=1
            r = max(0, self.speed)
            #*cos(scan_msg.angle_min+(scan_msg.angle_increment*-i))

            if r !=0:
                if(scan_msg.ranges[k]/r)< 0.4:
                    ack_msg= AckermannDriveStamped()
                    ack_msg.header.stamp= rospy.Time.now()
                    ack_msg.drive.steering_angle = 0
                    ack_msg.drive.speed=0

                    self.breakbool_pub.publish(True)
                    self.break_pub.publish(ack_msg)
        


def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()