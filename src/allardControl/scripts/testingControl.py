#!/usr/bin/env python
import sys
import math
import roslib
# roslib.load_manifest('robotiq_2f_gripper_control')

import rospy

import copy
import tf
import numpy as np
from control_msgs.msg import FollowJointTrajectoryActionGoal, GripperCommandActionGoal, GripperCommandGoal, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import PyKDL
from kinematics import inv_kin
import PyKDL
from urdf_parser_py.urdf import URDF
from geometry_msgs.msg import Pose, Quaternion

from pykdl_utils.kdl_kinematics import KDLKinematics
import tf.transformations as tf


robot = URDF.from_parameter_server()
kdl_kin = KDLKinematics(robot, 'base_link', 'wrist_3_link')
def ur2ros(ur_pose):
    """Transform pose from UR format to ROS Pose format.
Args:
    ur_pose: A pose in UR format [px, py, pz, rx, ry, rz] 
    (type: list)
Returns:
    An HTM (type: Pose).
"""

    # ROS pose
    ros_pose = Pose()

    # ROS position
    ros_pose.position.x = ur_pose[0]
    ros_pose.position.y = ur_pose[1]
    ros_pose.position.z = ur_pose[2]

    # Ros orientation
    # angle = sqrt(ur_pose[3] ** 2 + ur_pose[4] ** 2 + ur_pose[5] ** 2)
    # direction = [i / angle for i in ur_pose[3:6]]
    # np_T = tf.rotation_matrix(angle, direction)
    # np_q = tf.quaternion_from_matrix(np_T)
    np_q = tf.quaternion_from_euler(ur_pose[3], ur_pose[4], ur_pose[5])
    ros_pose.orientation.x = np_q[0]
    ros_pose.orientation.y = np_q[1]
    ros_pose.orientation.z = np_q[2]
    ros_pose.orientation.w = np_q[3]
    
    return ros_pose
# q = kdl_kin.random_joint_angles()
# pose = kdl_kin.forward(q) # forward kinematics (returns homogeneous 4x4 numpy.mat)


urdfLocation = '/home/allard/proj/src/allardControl/urdf/robot.urdf'
kinimaticsSolverLocation = '/home/allard/Documents/kdl-ik-solver/PyIkSolver'

if kinimaticsSolverLocation not in sys.path:
    sys.path.append(kinimaticsSolverLocation)

import solver
rospy.init_node('fakeController')
ik = solver.kdlSolver(urdfLocation, 'base_link', 'wrist_3_link', 10000, 16, 0.001)

pub = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size=10)
gp = rospy.Publisher('gripper/gripper_cmd/goal', GripperCommandActionGoal, queue_size=10)



joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

initialPose = solver.DoubleVector(6)


# initialPose[0] = 0
# initialPose[1] = -1.5
# initialPose[2] = 0.25
# initialPose[6] = 1


p_targ = solver.DoubleVector(7)

p_targ[0] = 0.55
p_targ[1] = -0.0
p_targ[2] = 0.25
p_targ[3] = 0
p_targ[4] = 0.707
p_targ[5] = 0
p_targ[6] = 0.707
# solver.normalizeQuaternion(p_targ)

des = [0, 0, 0, 0, 0, 0]

ths = inv_kin([-0.55, 0, 0.25, np.pi/2, 0, 0], des)
print(ur2ros([0.65, 0, 0.75, np.pi/2, 0, np.pi/2]))
ths = kdl_kin.inverse_search(ur2ros([0.65, 0, 0.1, np.pi/2, np.pi, 0]), 3, [-6, -3.1415, -np.pi,-2*np.pi,-2*np.pi,-2*np.pi], [6, 0, np.pi, 2*np.pi, 2*np.pi, 2*np.pi]) # inverse kinematics
print(kdl_kin.chain.getNrOfJoints())
q_targ = solver.DoubleVector(6)

# ik.solvePoseIk(initialPose, q_targ, p_targ, False)

posisions = [0 for i in range(len(joints))]
zeros = [0 for i in range(len(joints))]

posisions[0] = -0.5
posisions[1] = -1
posisions[2] = 1
posisions[3] = -1



gms = GripperCommandActionGoal()
goal = GripperCommand()
g = GripperCommandGoal()
goal.position=0.8
g.command = goal

gms.goal=g

# gp.publish(gms)

msg = JointTrajectory()
subMsg = JointTrajectoryPoint()
# print(msg)
# print(subMsg)

# print(type(zeros))
# solver.printDoubleVec(q_targ)
print(ths)
subMsg.positions = ths
subMsg.velocities = zeros
subMsg.accelerations = zeros
msg.joint_names = joints
msg.points=[subMsg]
rate=rospy.Rate(1)
subMsg.time_from_start = rospy.Duration(0.5)




for i in range(2):
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    gp.publish(gms)
    rate.sleep()


try:
    # rospy.spin()
    pass
except rospy.ROSInterruptException:
    pass