#!/usr/bin/env python

# Author: Lasitha Wijayarathne
# This node simulates the pulsating behaviour of the table

import numpy as np
import scipy
import rospy
from geometry_msgs.msg import Pose, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint 
from std_msgs.msg import Float64
from trac_ik_python.trac_ik import IK


# starting time
time_start = 0.0

def follow_timed_joint_trajectory(positions, velocities, times):
 
    jt = JointTrajectory()
    jt.joint_names = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
    jt.header.stamp = rospy.Time.now()
 
    for (position, velocity, time) in zip(positions, velocities, times):
        jtp = JointTrajectoryPoint()
        jtp.positions = position
        jtp.velocities = velocity
        jtp.time_from_start = rospy.Duration(time)
        jt.points.append(jtp)
 
    pub_controller_tr.publish(jt)


def ik_solve(base_frame, end_frame, seed_state):

	ik_solver = IK(base_frame, end_frame)

	ik_solution = ik_solver.get_ik(seed_state,
					                -0.3, -0.3, 1.1,
					                0.0, 0.0, 0.0, 1.0,
					                0.01, 0.01, 0.01,  # X, Y, Z bounds
					                0.1, 0.1, 0.1)  
	return ik_solution

def move_table(pos_1, pos_2):
	_pub_pos_pr.publish(pos_1)	
	_pub_pos_tilt.publish(pos_2)

if __name__ == '__main__':

	rospy.init_node("hold_initial_pose")
	
	pub_controller_tr = rospy.Publisher('/iiwa/EffortJointInterface_trajectory_controller/command', JointTrajectory,
	                             queue_size=1)


	r = rospy.Rate(100) # 100hz 

	# create a sript to run sinusoidal signals forever
	t = time_start


	seed_state = [0.0] * 7
	ik_sol = ik_solve('iiwa_link_0', 'iiwa_link_7', seed_state)
	print(ik_sol)

	times      = [0.5]
	positions  = [ik_sol]
	velocities = [[]]

	while not rospy.is_shutdown():
	# 	seed_state = [0.0] * ik_solver.number_of_joints

		follow_timed_joint_trajectory(positions, velocities, times)
	# 	r.sleep()	
