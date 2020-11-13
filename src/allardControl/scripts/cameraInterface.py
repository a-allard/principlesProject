#!/usr/bin/env python

import rospy
import cv2, cv_bridge
from sensor_msgs.msg import Image
import numpy as np



class cameraInterface(object):
    
    
    
    def __init__(self, liveScan=True):
        self.interface = rospy.Subscriber('ur10/usbcam/image_raw', Image, self.__updateImage__, queue_size=3)
        self.cameraData = None
        self.liveScan = liveScan
        self.blockLocations = []
        self.blockColors = []
        self.bridge = cv_bridge.CvBridge()
        
    
    def __updateImage__(self, data):
        self.cameraData = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        if self.liveScan:
            self.scanImage()
    
    def scanImage(self):
        
        hsv = cv2.cvtColor(self.cameraData, cv2.COLOR_BGR2HSV)
        lower_red = np.array([ 0,  100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        img, cnts, _= cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        image = self.cameraData.copy()
        if(len(cnts) > 0):
            self.blockLocations = []
            self.blockColors = []
        # print(self.cameraData.shape)
        for i, c in enumerate(cnts):
            M = cv2.moments(c)
            if(M['m00'] > 0):
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                area = cv2.contourArea(c)
                
                self.blockLocations.append((cx, cy))

                cv2.circle(image, (cx, cy), 5, (0,0,0), -1)
                cv2.putText(image, "({}, {})".format(int(cx), int(cy)), (int(cx-5), int(cy+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.drawContours(image, cnts, -1, (255, 255, 255),1)


        cv2.namedWindow("window", 1)
        cv2.imshow("window", image)
        cv2.waitKey(1)