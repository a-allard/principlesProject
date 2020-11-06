#!/usr/bin/env python
import rospy
from control_msgs.msg import GripperCommandActionGoal, GripperCommandGoal, GripperCommand

class gripper(object):
    
    def __init__(self):
        self.interface = rospy.Publisher('gripper/gripper_cmd/goal', GripperCommandActionGoal, queue_size=10)
        self.rate1Hz = rospy.Rate(1)
        self.gripperCommandGoal = GripperCommandActionGoal()
        self.gripperCmd = GripperCommand()
        self.goal = GripperCommandGoal()
        self.goal.command = self.gripperCmd
        self.gripperCommandGoal.goal = self.goal
        self.isOpen = True
    
    def __publish__(self):
        for i in range(2):
            self.interface.publish(self.gripperCommandGoal)
            self.rate1Hz.sleep()
        
    def openGripper(self):
        self.gripperCommandGoal.goal.command.position = 0
        self.__publish__()
        self.isOpen = True
    
    def closeGripper(self):
        self.gripperCommandGoal.goal.command.position = 0.4
        self.__publish__()
        self.gripperCommandGoal.goal.command.position = 0.45
        self.__publish__()
        self.isOpen = False
        