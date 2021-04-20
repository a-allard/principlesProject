#!/usr/bin/env python
import rospy
from control_msgs.msg import GripperCommandActionGoal, GripperCommandGoal, GripperCommand
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from ur10_gripper import trigger_gripper
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from gazebo_msgs.srv import GetWorldProperties, GetWorldPropertiesRequest, GetWorldPropertiesResponse, \
    GetModelState, GetModelStateRequest, GetModelStateResponse, \
        GetLinkState, GetLinkStateRequest, GetLinkStateResponse
import time

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


class gripper2(object):
    def __init__(self):
        self.attServ = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.detServ = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        self.worldPropsServ = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        self.modelStateServ = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.linkStateServ = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self.currentModel1 = 'robot'
        self.currentModel2 = None
        self.link1 = 'wrist_3_link'
        self.link2 ='block'
        # self.gripperMsg = AttachRequest()
        # self.gripperMsg.link_name_1 = self.link1
        # self.gripperMsg.link_name_2 = self.link2
        # self.gripperMsg.model_name_1 = self.currentModel1
        self.isGripping = False
        self.gazeboModelsMsg = GetWorldPropertiesRequest()
        self.gazeboModelStateMsg = GetModelStateRequest()
        self.gazeboModelStateMsg.model_name = 'block_0'
        self.gazeboLinkStateMsg = GetLinkStateRequest()
        self.gazeboLinkStateMsg.link_name = 'wrist_3_link'
        
    def openGripper(self):
        if not self.isGripping:
            return
        self.currentModel2 = self.findClosestBlock()
        self.gripperMsg = AttachRequest()
        self.gripperMsg.link_name_1 = self.link1
        self.gripperMsg.link_name_2 = self.link2
        self.gripperMsg.model_name_1 = self.currentModel1
        self.gripperMsg.model_name_2 = self.currentModel2
        self.detServ.call(self.gripperMsg)
        rospy.sleep(1)
        self.isGripping = False
    
    def closeGripper(self):
        if self.isGripping:
            return
        self.currentModel2 = self.findClosestBlock()
        self.gripperMsg = AttachRequest()
        self.gripperMsg.link_name_1 = self.link1
        self.gripperMsg.link_name_2 = self.link2
        self.gripperMsg.model_name_1 = self.currentModel1
        self.gripperMsg.model_name_2 = self.currentModel2
        self.attServ.call(self.gripperMsg)
        self.isGripping = True
        rospy.sleep(1)
    
    def findClosestBlock(self):
        models = self.worldPropsServ.call(self.gazeboModelsMsg).model_names
        positions = []
        mods = []
        for m in models:
            if not 'block' in m.lower():
                continue
            self.gazeboModelStateMsg.model_name = m
            positions.append(self.modelStateServ.call(self.gazeboModelStateMsg).pose.position)
            mods.append(m)
        link = self.linkStateServ.call(self.gazeboLinkStateMsg).link_state.pose.position
        ers = [(link.x - s.x) ** 2 + (link.y - s.y) ** 2 + (link.z - s.z) ** 2 for s in positions]
        
        return mods[ers.index(min(ers))]