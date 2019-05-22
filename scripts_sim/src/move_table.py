#!/usr/bin/env python

# Author: Lasitha Wijayarathne
# This node simulates the pulsating behaviour of the table

import numpy as np
import scipy
import rospy
from geometry_msgs.msg import Pose, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint 
from std_msgs.msg import Float64

# starting time
time_start = 0.0

# build a simple trajectory from the start to end position
#   - for the FS100, we can get by with a simple 2-point trajectory
#   - the controller handles the necessary accel/decel to make smooth motion
def build_traj(start, end, duration):
	# if sorted(start.name) <> sorted(end.name):
	# 	rospy.logerr('Start and End position joint_names mismatch')
	# 	raise

	# assume the start-position joint-ordering
	joint_names = start.name

	start_pt = JointTrajectoryPoint()
	start_pt.positions = start.position
	start_pt.velocities = [0]*len(start.position)
	start_pt.time_from_start = rospy.Duration(1.0)

	end_pt = JointTrajectoryPoint()
	for j in joint_names:
		idx = end.name.index(j)
		end_pt.positions.append(end.position[idx])  # reorder to match start-pos joint ordering
		end_pt.velocities.append(0)
		end_pt.time_from_start = rospy.Duration(duration)

	return JointTrajectory(joint_names=joint_names, points=[start_pt, end_pt])

def move_table(pos_1, pos_2):
	_pub_pos_pr.publish(pos_1)	
	_pub_pos_tilt.publish(pos_2)

if __name__ == '__main__':

	rospy.init_node("table_pulsate")
	
	_pub_pos_pr = rospy.Publisher('/iiwa/EffortJointInterface_table_joint_controller/command', Float64,
	                             queue_size=1)
	_pub_pos_tilt = rospy.Publisher('/iiwa/EffortJointInterface_table_joint_tilt_controller/command',Float64,
                             queue_size=1)

	r = rospy.Rate(100) # 100hz 

	# create a sript to run sinusoidal signals forever
	t = time_start
	while not rospy.is_shutdown():
		j1_pos = 0.08*np.sin(2*np.pi*t) + 0.3
		j2_pos = 0.3*np.cos(2*np.pi*t)
		move_table(j1_pos, j2_pos)
		t += 0.01
		r.sleep()	

	# rospy.spin()
