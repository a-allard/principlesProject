#!/usr/bin/env python
import rospy
from control_msgs.msg import GripperCommandActionGoal, GripperCommandGoal, GripperCommand
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from ur10_gripper import trigger_gripper

class gripper(object):
    
    def __init__(self):
        # self.interface = rospy.Publisher('gripper/gripper_cmd/goal', GripperCommandActionGoal, queue_size=10)
        # self.interface = rospy.Publisher('ur10/gripper/grasping', Bool, queue_size=1)
        self.rate1Hz = rospy.Rate(1)
        self.gripperCommandGoal = GripperCommandActionGoal()
        self.gripperCmd = GripperCommand()
        self.goal = GripperCommandGoal()
        self.goal.command = self.gripperCmd
        self.gripperCommandGoal.goal = self.goal
        self.msg = Bool()
        self.msg.data = True
        self.isOpen = True
    
    # def __publish__(self):
    #     for i in range(2):
    #         # self.interface.publish(self.gripperCommandGoal)
    #         self.interface.publish(self.msg)
    #         self.rate1Hz.sleep()
        
    def openGripper(self):
        self.gripperCommandGoal.goal.command.position = 0
        self.msg.data = False
        trigger_gripper(False)
        self.rate1Hz.sleep()
        # self.__publish__()
        self.isOpen = True
    
    def closeGripper(self):
        # self.gripperCommandGoal.goal.command.position = 0.4
        # self.__publish__()
        self.gripperCommandGoal.goal.command.position = 0.45
        self.msg.data = True
        trigger_gripper(True)
        self.rate1Hz.sleep()
        # self.__publish__()
        self.isOpen = False
        