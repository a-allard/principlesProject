#!/usr/bin/env python

import rospy
import rospkg
from ur10arm import ur10Arm
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
import sys
from random import randint, uniform, seed
import numpy as np
import time



class finalProject(object):
    def __init__(self, random=False, numBlocks=3):
        rospy.init_node("mainController")
        seed(time.time())
        self.arm = ur10Arm()
        self.numBlocks = numBlocks
        self.blockList = []  # list of dicts {'color':'blue, 'loc':(0,0,0), 'name':'block0'} Example
        self.binList = []
        self.blockConter = 0
        self.rlim = (0.4, 0.9)
        self.thlim = (-np.pi/3, np.pi/3)
        self.colors = ['Blue', 'Green', 'Red']
        self.binLocs = [{'x': -0.5, 'y':-0.5}, {'x': -0.5, 'y':0}, {'x':-0.5, 'y':0.5}]
        self.gripperTouching = 0.07# 0.14
        self.xOffset = 0.0#0.055
        self.yOffset = 0.0#0.028
        self.initialPose = [-0.85, 0, 0.85, 0,np.pi,np.pi ] #-np.pi/2, np.pi
        self.preferedAngs = [[-6, -2.5415, -np.pi,-2*np.pi,-2*np.pi,-2*np.pi], [6, -0.3, np.pi, 2*np.pi, 2*np.pi, 2*np.pi]]
        self.preferedDumpingAngs = [[-6, -2.5415, -np.pi,-2*np.pi,-2*np.pi,-2*np.pi], [6, -0.3, np.pi, 2*np.pi, 2*np.pi, 2*np.pi]]
        print('Setting arm to initial pose')
        self.setArmInitialPose()
        print('Building blocks...')
        self.initBlocks(random)
        print('Now the bins...')
        self.initBins()
        rospy.on_shutdown(self.removeAllBlocks)
        time.sleep(1)
    
    def initBins(self):
        for c, loc in zip(self.colors, self.binLocs):
            self.makeBin(loc, c)
    
    def initBlocks(self, random=False):
        for i in range(self.numBlocks):
            r = uniform(*self.rlim)
            th = uniform(*self.thlim)
            pos = {'x': r * np.cos(th), 'y': r * np.sin(th)}
            c = randint(0, len(self.colors))
            self.makeBlock(pos, self.colors[c-1])
        
    
    def makeBlock(self, position, color):
        block_xml = ''
        path = rospkg.RosPack().get_path('allardControl') + '/'
        with open(path + 'urdf/block.urdf') as bf:
            block_xml = bf.read().replace('\n', '').replace('Red', color)#.replace('origin xyz="0.0 0.0 0.05"', 'origin xyz="{0:.5f} {1:.5f} 0.05"'.format(position['x'], position['y']))
        block_reference_frame = 'world'
        block_pose=Pose(position=Point(x=position['x'], y=position['y'], z=0))
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        blockName = 'block_{0}'.format(self.blockConter)
        if self.makeObject(blockName, block_xml, block_pose, block_reference_frame):
            self.blockList.append({'color': color, 'loc': position, 'name': blockName})
        
        self.blockConter += 1
    
    def makeBin(self, position, color):
        block_xml = ''
        path = rospkg.RosPack().get_path('allardControl') + '/'
        with open(path + 'urdf/bin.urdf') as bf:
            block_xml = bf.read().replace('\n', '').replace('Blue', color)
        block_reference_frame = 'world'
        block_pose=Pose(position=Point(x=position['x'], y=position['y'], z=0))
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        blockName = 'bin_{0}'.format(color)
        if self.makeObject(blockName, block_xml, block_pose, block_reference_frame):
            self.binList.append({'color': color, 'loc': position, 'name': blockName})
        
        
    def makeObject(self, blockName, block_xml, block_pose, block_reference_frame):
        suc = False
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf = spawn_urdf(blockName, block_xml, "/",
                                block_pose, block_reference_frame)
            suc = True
        except rospy.ServiceException:
            rospy.logerr("Spawn URDF service call failed: {0}".format(sys.exc_info()[0]))
        return suc
    
    def removeAllBlocks(self):
        print('Deleteing everying dear to you...')
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            for block in self.blockList:
                resp_delete = delete_model(block['name'])
            self.blockList = []
            for bin in self.binList:
                resp_delete = delete_model(bin['name'])
            self.binList = []
        except rospy.ServiceException:
            rospy.loginfo("Delete Model service call failed: {0}".format(sys.exc_info()[0]))
    
    
    def setArmInitialPose(self):
        self.arm.setArmPosition(self.initialPose, self.preferedAngs)
        self.arm.gripper.openGripper()
    
    def __del__(self):
        self.arm.gripper.openGripper()
        self.removeAllBlocks()
    
    def findBlock(self, camera):
        if camera > -1:
            lc = self.blockList[camera]['loc'].copy()
            lc['x'] = (lc['x'] + self.xOffset) * 1
            lc['y'] = (lc['y'] + self.yOffset) * 1
            return lc, self.blockList[camera]['color']
        else:
            color, loc = self.arm.centerBlock()
            print(loc)
            print(color)
            return loc, color
    
    def pickUpBlock(self, location):
        self.arm.setArmPosition([location['x'], location['y'], 0.85, 0,np.pi,np.pi])
        # rospy.sleep(0.5)
        # self.arm.setArmPosition([location['x'], location['y'], self.gripperTouching+0.16, 0, np.pi, 0])
        
        self.arm.setArmPosition([location['x'], location['y'], self.gripperTouching, 0,np.pi,np.pi])
        print('*Chomp!!*')
        self.arm.gripper.closeGripper()
        # self.arm.setArmPosition([location['x'], location['y'], self.gripperTouching, np.pi/2, np.pi, 0])
        # self.arm.setArmPosition([location['x'], location['y'], self.gripperTouching-0.002, np.pi/2, np.pi, 0])
        # rospy.sleep(0.5)
        # self.arm.gripper.closeGripper()
        # self.arm.setArmPosition([location['x'], location['y'], self.gripperTouching+0.1, np.pi/2, np.pi, 0])
        # rospy.sleep(0.5)
        self.arm.setArmPosition([location['x'], location['y'], 0.85, 0,np.pi,np.pi])
        # rospy.sleep(0.5)
    
    def moveToBin(self, color):
        print('Bin it baby!\nBlock color: {0}'.format(color))
        loc = self.binLocs[self.colors.index(color)]
        lc = loc.copy()
        lc['x'] = (lc['x'] + self.xOffset) * -1
        lc['y'] = (lc['y'] + self.yOffset) * -1
        print('Headed for the bin...')
        self.arm.setArmPosition([lc['x'], lc['y'], 0.85, 0,np.pi,np.pi], self.preferedDumpingAngs)
        print('Squatting...')
        self.arm.setArmPosition([lc['x'], lc['y'], 0.55, 0,np.pi,np.pi], self.preferedDumpingAngs)
        print('*Bonk!!*')
        self.arm.gripper.openGripper()
    
    def cleanUpBlocks(self, useCam=False):
        for i in range(self.numBlocks):
            if not useCam:
                location, color = self.findBlock(i)
            else:
                # location, color = self.findBlock(-1)
                # self.arm.setArmPosition([location['x'], location['y'], 0.75/2, np.pi/2, np.pi, 0])
                location, color = self.findBlock(-1)
                # self.arm.setArmPosition([location['x'], location['y'], self.gripperTouching, np.pi/2, np.pi, 0])
                # self.arm.gripper.closeGripper()
            self.pickUpBlock(location)
            self.moveToBin(color)
            self.setArmInitialPose()






if __name__ == '__main__':
    fp = finalProject()
    fp.cleanUpBlocks(True)
    while not rospy.is_shutdown():
        rospy.sleep(0.1)