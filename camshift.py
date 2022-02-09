#!/usr/bin/env python3
from __future__ import print_function
from email.mime import image
from glob import glob
import sys
from setuptools import setup
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

setup=True
class cam_shift:

    def __init__(self):
        print("init")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.callback)
    
    def callback(self,data):
        global setup
        global roi
        global hsv_roi
        global mask
        global roi_hist
        global term_crit
        global track_window
        # Recuperation du message image et transformation en image opencv en utilisant "Bridge"
        try:
            cap = self.bridge.imgmsg_to_cv2(data, "bgr8") 
        except CvBridgeError as e:
            print(e)

        if (setup): #Setup a faire une fois au debut

            # Position initiale du rectangle
            r,h,c,w = 250,90,400,125
            track_window = (c,r,w,h)

            # set up du rectangle pour le tracking
            roi = cap[r:r+h, c:c+w]
            hsv_roi =  cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_roi, np.array((0., 60.,32.)), np.array((180.,255.,255.)))
            roi_hist = cv2.calcHist([hsv_roi],[0],mask,[180],[0,180])
            cv2.normalize(roi_hist,roi_hist,0,255,cv2.NORM_MINMAX)

            # SCritere d'arret, 10 iterations ou bouger de 1 pt
            term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )

            setup=False
        # Image processing
        ret = True
        hsv = cv2.cvtColor(cap, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([hsv],[0],roi_hist,[0,180],1)

        # Appliquer l'algo de Camshift
        ret,track_window = cv2.CamShift(dst, track_window, term_crit)

        # Dessiner le rectangle 
        pts = cv2.boxPoints(ret)
        pts = np.int0(pts)
        img2 = cv2.polylines(cap,[pts],True, 255,2)
        cv2.imshow('img2',img2)
        k = cv2.waitKey(60) & 0xff #affichage de l'image en 60 fps  
        if k == 27:
            return
        else:
            cv2.imwrite(chr(k)+".jpg",img2)

def main(args):
    ic = cam_shift()
    rospy.init_node('cam_shift', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)