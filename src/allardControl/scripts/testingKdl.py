#!/usr/bin/env python

import PyKDL
from urdf_parser_py.urdf import URDF

from pykdl_utils.kdl_kinematics import KDLKinematics
robot = URDF.from_parameter_server()
kdl_kin = KDLKinematics(robot, 'base_link', 'wrist_3_link')
q = kdl_kin.random_joint_angles()
pose = kdl_kin.forward(q) # forward kinematics (returns homogeneous 4x4 numpy.mat)
q_ik = kdl_kin.inverse(pose, q+0.3) # inverse kinematics

class ur10_Kimematics(object):
    
    def __init__(self):
        self.urdfLocation = '/home/allard/proj/src/allardControl/urdf/robot.urdf'
        
    
    