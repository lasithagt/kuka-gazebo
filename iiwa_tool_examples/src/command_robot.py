#!/usr/bin/env python

import os, sys, roslib, rospy, argparse, time

# Include directory path for the compiled pybindings library
libdir = os.path.join(os.path.expanduser('~'), 'catkin_ws', 'devel_isolated', 'lib')
sys.path.insert(0, libdir)
libdir = os.path.join(os.path.expanduser('~'), 'catkin_ws', 'devel', 'lib')
sys.path.insert(0, libdir)

import numpy as np
import cPickle as pickle

import pybindings
from iiwa_msgs.msg import JointPosition, JointPositionVelocity, JointVelocity, JointTorque
from geometry_msgs.msg import PoseStamped


def readFile(filename):
    with open(filename, 'rb') as f:
        return [[float(v) for v in line.strip().split()] for line in f.readlines()]
        # return pickle.load(f)

if __name__ == '__main__':
    rospy.init_node('robot_position_command')
    rate = rospy.Rate(60.0)

    iiwa = pybindings.iiwaRosGazebo()
    nodeHandle = iiwa.initPy(60.0)
    iiwa.init(nodeHandle)

    data_qd = readFile(os.path.join(os.path.expanduser('~'), 'catkin_ws', 'src', 'kuka-iiwa', 'iiwa_tool_examples', 'src', 'data', 'desired_velocity.txt'))

    for d in data_qd:
        velocity = np.array(d) / 10.0
        print velocity
        iiwa.setJointVelocityPy(velocity)
        rate.sleep()

