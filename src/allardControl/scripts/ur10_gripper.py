#!/usr/bin/env python



import rospy, sys, numpy as np
import geometry_msgs.msg
import moveit_msgs.msg
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty
import time

def gripper_status(msg):
    if msg.data:
        return True
        # print('gripper status = {}'.format(msg.data))

def gripper_on():
    # Wait till the srv is available
    # rospy.wait_for_service('/ur10/vacuum_gripper/on', 1)
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur10/vacuum_gripper/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException:
        print("Service call failed: %s" % sys.exc_info()[0])

def gripper1_on():
    # Wait till the srv is available
    # rospy.wait_for_service('/ur10/vacuum_gripper1/on', 1)
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur10/vacuum_gripper1/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException:
        print("Service call failed: %s" % sys.exc_info()[0])

def gripper2_on():
    # Wait till the srv is available
    # rospy.wait_for_service('/ur10/vacuum_gripper2/on', 1)
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur10/vacuum_gripper2/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException:
        print("Service call failed: %s" % sys.exc_info()[0])

def gripper3_on():
    # Wait till the srv is available
    # rospy.wait_for_service('/ur10/vacuum_gripper3/on', 1)
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur10/vacuum_gripper3/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException:
        print("Service call failed: %s" % sys.exc_info()[0])

def gripper4_on():
    # Wait till the srv is available
    # rospy.wait_for_service('/ur10/vacuum_gripper4/on', 1)
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur10/vacuum_gripper4/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException:
        print("Service call failed: %s" % sys.exc_info()[0])

def gripper5_on():
    # Wait till the srv is available
    # rospy.wait_for_service('/ur10/vacuum_gripper5/on', 1)
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur10/vacuum_gripper5/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException:
        print("Service call failed: %s" % sys.exc_info()[0])

def gripper6_on():
    # Wait till the srv is available
    # rospy.wait_for_service('/ur10/vacuum_gripper6/on', 1)
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur10/vacuum_gripper6/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException:
        print("Service call failed: %s" % sys.exc_info()[0])

def gripper7_on():
    # Wait till the srv is available
    # rospy.wait_for_service('/ur10/vacuum_gripper7/on', 1)
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur10/vacuum_gripper7/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException:
        print("Service call failed: %s" % sys.exc_info()[0])

def gripper8_on():
    # Wait till the srv is available
    # rospy.wait_for_service('/ur10/vacuum_gripper8/on', 1)
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur10/vacuum_gripper8/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException:
        print("Service call failed: %s" % sys.exc_info()[0])


def gripper_off():
    # rospy.wait_for_service('/ur10/vacuum_gripper/off', 1)
    try:
        turn_off = rospy.ServiceProxy('/ur10/vacuum_gripper/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException:
        print("Service call failed: %s" % sys.exc_info()[0])

def gripper1_off():
    # rospy.wait_for_service('/ur5/vacuum_gripper1/off', 1)
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper1/off', Empty)
        resp = turn_off()
        del turn_off
        return resp
    except rospy.ServiceException:
        print("Service call failed: %s" % sys.exc_info()[0])

def gripper2_off():
    # rospy.wait_for_service('/ur10/vacuum_gripper2/off', 1)
    try:
        turn_off = rospy.ServiceProxy('/ur10/vacuum_gripper2/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException:
        print("Service call failed: %s" % sys.exc_info()[0])

def gripper3_off():
    # rospy.wait_for_service('/ur10/vacuum_gripper3/off', 1)
    try:
        turn_off = rospy.ServiceProxy('/ur10/vacuum_gripper3/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException:
        print("Service call failed: %s" % sys.exc_info()[0])

def gripper4_off():
    # rospy.wait_for_service('/ur10/vacuum_gripper4/off', 1)
    try:
        turn_off = rospy.ServiceProxy('/ur10/vacuum_gripper4/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException:
        print("Service call failed: %s" % sys.exc_info()[0])

def gripper5_off():
    # rospy.wait_for_service('/ur10/vacuum_gripper5/off', 1)
    try:
        turn_off = rospy.ServiceProxy('/ur10/vacuum_gripper5/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException:
        print("Service call failed: %s" % sys.exc_info()[0])

def gripper6_off():
    # rospy.wait_for_service('/ur10/vacuum_gripper6/off', 1)
    try:
        turn_off = rospy.ServiceProxy('/ur10/vacuum_gripper6/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException:
        print("Service call failed: %s" % sys.exc_info()[0])

def gripper7_off():
    # rospy.wait_for_service('/ur10/vacuum_gripper7/off', 1)
    try:
        turn_off = rospy.ServiceProxy('/ur10/vacuum_gripper7/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException:
        print("Service call failed: %s" % sys.exc_info()[0])

def gripper8_off():
    # rospy.wait_for_service('/ur10/vacuum_gripper8/off', 1)
    try:
        turn_off = rospy.ServiceProxy('/ur10/vacuum_gripper8/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException:
        print("Service call failed: %s" % sys.exc_info()[0])

def trigger(msg):
    gripper_trigger = msg.data
    if gripper_trigger:
        # print('grippers on!')
        gripper_on()
        gripper1_on()
        gripper2_on()
        gripper3_on()
        gripper4_on()
        gripper5_on()
        gripper6_on()
        gripper7_on()
        gripper8_on()

    else:
        gripper_off()
        gripper1_off()
        gripper2_off()
        gripper3_off()
        gripper4_off()
        gripper5_off()
        gripper6_off()
        gripper7_off()
        gripper8_off()

def trigger_gripper(msg):
    
    if msg:
        gripper_on()
        gripper1_on()
        # time.sleep(0.1)
        gripper2_on()
        # time.sleep(0.1)
        gripper3_on()
        # time.sleep(0.1)
        gripper4_on()
        # time.sleep(0.1)
        gripper5_on()
        # time.sleep(0.1)
        gripper6_on()
        # time.sleep(0.1)
        gripper7_on()
        # time.sleep(0.1)
        gripper8_on()

    else:
        gripper_off()
        # time.sleep(0.1)
        gripper1_off()
        # time.sleep(0.1)
        gripper2_off()
        # time.sleep(0.1)
        gripper3_off()
        # time.sleep(0.1)
        gripper4_off()
        # time.sleep(0.1)
        gripper5_off()
        # time.sleep(0.1)
        gripper6_off()
        # time.sleep(0.1)
        gripper7_off()
        # time.sleep(0.1)
        gripper8_off()

# rospy.init_node("ur10_gripper", anonymous=False)

# gripper_status_sub = rospy.Subscriber('/ur10/gripper/grasping', Bool, trigger, queue_size=1)

# cxy_sub = rospy.Subscriber('cxy1', Tracker, trigger, queue_size=1)

# rospy.spin()