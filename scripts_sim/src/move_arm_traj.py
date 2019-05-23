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
import transformations as trans
import math

PI = 3.14
# starting time
time_start = 0.0
x_b = 0.001; y_b = 0.001; z_b = 0.001
r_x = 0.01;  r_y = 0.01;  r_z = 0.01

# initialize the solver
# ik_solver = IK('iiwa_link_0', 'kuka_fitting_ee_')
ik_solver = IK('iiwa_link_0', 'iiwa_link_7')

# set the joint limits
# lower_bounds = np.array([])
# ik_solver.set_joint_limits(lower_bounds, upper_bounds):

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
 
 	# print('Publishing data...')
    pub_controller_tr.publish(jt)


def ik_solve(base_frame, end_frame, seed_state, pose):
	
	
	ik_solution = ik_solver.get_ik(seed_state, 
									pose[0], pose[1], pose[2],
						                pose[3], pose[4], pose[5], pose[6],
					                x_b, y_b, z_b,  # X, Y, Z bounds
					                r_x, r_y, r_z)  

	return ik_solution


def generate_desired_path():
	r = 0.0254 # radius of the sphere
	n = 100
	phi = np.linspace(0.0, np.pi, num=n); theta = np.linspace(0.0, 0.0, num=n)

	# generate points with orientation
	points = np.array([r * np.cos(phi) * np.cos(theta), r * np.cos(phi) * np.sin(theta), r * np.sin(phi)])
	# print(points[:,1])

	quats = np.zeros((4,n))
	jpos  = np.zeros((8,n))

	orien = lambda p : [[-2*p[0]],[-2*p[1]],[-2*p[2]]]

	seed_state = [0.0] * 7

	# transformation from the real kuka base to the location of the workplace
	T_b = np.identity(4)
	R_b = trans.rotation_matrix(np.pi, [0, 1, 0], [0, 0, 1])

	T_b[:3,:3] = R_b[:3, :3]
	T_b[:3,-1] = np.array([0.,0.1,1.01])
	
	v0 = np.dot(T_b[:3, :3], np.array([[0,0,-1]]).T)

	for i in range(n):
		v1 = np.dot(R_b[:3,:3], np.array([orien(points[:,i])]).T) 
		print(v1.T.flatten())
		v2 = np.dot(R_b[:3,:3], np.array([points[:,i]]).T)  + np.array([[0.],[0.1],[1.01]])

		# check this multiplication
		# M = np.multiply(R_b[3:,3:], trans.rotation_matrix(trans.angle_between_vectors(v0.T.flatten().tolist(), v1.T.flatten().tolist()), trans.vector_product(v0.T.flatten().tolist(), v1.T.flatten().tolist())).T)
		M = trans.rotation_matrix(trans.angle_between_vectors(v0.T.flatten(), v1.T.flatten()), trans.vector_product(v0.T.flatten(), v1.T.flatten()))
		quat = trans.quaternion_from_matrix(M,  isprecise=True)
		# print(quat)
		quat = np.concatenate((np.array(quat[1:]),np.array([quat[0]])),axis=0)

		quats[:,i] = quat
		pose = np.concatenate((v2.T.flatten(),quat),axis=0)
		jpos[:,i]  = ik_solve('iiwa_link_0', 'iiwa_link_7', seed_state, pose)

		# print(pose)
		# print(jpos[:,i])
		# set the new seed state
		if (not math.isnan(jpos[0,i])):
			seed_state = jpos[:,i]

	# print(points)
	print(quats.T)
	return jpos


def move_to_inital_pose(seed_state, pose):

	print(pose)
	ik_sol = ik_solve('iiwa_link_0', 'iiwa_link_7', seed_state, pose)

	# if (ik_sol is None):
	print(ik_sol)

	times      = [5]
	positions  = [ik_sol]
	velocities = [[]]

	# publish to get to the initial pose
	follow_timed_joint_trajectory(positions, velocities, times)


if __name__ == '__main__':

	rospy.init_node("move_kuka_arm")
	
	pub_controller_tr = rospy.Publisher('/iiwa/EffortJointInterface_trajectory_controller/command', JointTrajectory,
	                             queue_size=1)

	r = rospy.Rate(100) # 100hz 

	# create a sript to run sinusoidal signals forever
	t = time_start


	seed_state = [0.0] * 7
	# initial pose to start
	initial_pose = [0.1,0.0,1.1,0.0,0.0,0.0,1.0]

	
	jtraj = generate_desired_path()
	print(jtraj)
	print(jtraj.T.shape)

	t = np.linspace(0.1,20,100)


	# while not rospy.is_shutdown():
	move_to_inital_pose(seed_state, initial_pose)

	rospy.sleep(3)

	times      = [3]
	positions  = [jtraj[:,0].T]
	velocities = [[]]
	follow_timed_joint_trajectory(positions, velocities, times)

	rospy.sleep(4)

	for i in range(100):
		times      = [t[i]]
		positions  = [jtraj[:,i].T]
		velocities = [[]]
		
		if (not math.isnan(positions[0][0])):
			follow_timed_joint_trajectory(positions, velocities, times)
			print(positions[0][0])	
			print(i)
			rospy.sleep(0.1)

	rospy.sleep(10)
