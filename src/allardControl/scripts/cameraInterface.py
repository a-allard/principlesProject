#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image



class cameraInterface(object):
    
    
    
    def __init__(self, liveScan=False):
        rospy.init_node('camera_processor')
        self.interface = rospy.Subscriber('ur10/usbcam/image_raw', Image, self.__updateImage__, queue_size=3)
        self.cameraData = None
        self.liveScan = liveScan
        self.blockLocations = []
        self.blockColors = []
        
    
    def __updateImage__(self, data):
        self.cameraData = data
        if self.liveScan:
            self.scanImage()
    
    def scanImage(self):
        self.blockLocations = []
        self.blockColors = []