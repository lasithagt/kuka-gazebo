#include <iiwa_ros.h>
#include <cmath>

#include <fstream>
#include <vector>
#include <iostream>
#include <Eigen/StdVector>

#include <iomanip>
#include <eigen3/Eigen/Dense>

#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

#include <iiwa_ros/conversions.h>


int main (int argc, char **argv) {
	
	// Initialize ROS
	ros::init(argc, argv, "SaveData");
	ros::NodeHandle nh("~");

	// ROS spinner.
	ros::AsyncSpinner spinner(1);
	spinner.start();

	iiwa_ros::iiwaRosReal my_iiwa;
	my_iiwa.init(nh);

	// Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
	bool use_cartesian_command;
	nh.param("use_cartesian_command", use_cartesian_command, false);

	// Dynamic parameter to choose the rate at wich this node should run
	double ros_rate;
	nh.param("ros_rate", ros_rate, 100.0); // 0.1 Hz = 10 seconds
	ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

	geometry_msgs::PoseStamped command_cartesian_position;
	iiwa_msgs::JointPosition command_joint_position;
	iiwa_msgs::JointVelocity command_joint_velocity;

	// States to measure
	iiwa_msgs::JointPosition state_joint_position;
	iiwa_msgs::JointVelocity state_joint_velocity;
	iiwa_msgs::JointTorque state_joint_torque;

	std::cout << "Here" << std::endl;
	// bool new_pose = false, motion_done = false;

	// int direction = 1;

	// States to measure in the matrix

	std::ofstream data;
	data.open ("JointData_new.txt");

	int init = 0;

	while (ros::ok()) {
		std::cout << "Here" << std::endl;

		my_iiwa.getJointPosition(state_joint_position);
		my_iiwa.getJointVelocity(state_joint_velocity);
		my_iiwa.getJointTorque(state_joint_torque);
		my_iiwa.getJointCommVelocity(command_joint_velocity); 

		data << state_joint_position.position.a1 << " " << state_joint_position.position.a2 << " " << state_joint_position.position.a3 << " " << \
		state_joint_position.position.a4 << " " << state_joint_position.position.a5 << " " << state_joint_position.position.a6 << " " << state_joint_position.position.a7 << " " << state_joint_velocity.velocity.a1 << " " << state_joint_velocity.velocity.a2 << " " << state_joint_velocity.velocity.a3 << " " << \
		state_joint_velocity.velocity.a4 << " " << state_joint_velocity.velocity.a5 << " " << state_joint_velocity.velocity.a6 << " " << state_joint_velocity.velocity.a7 << " " << command_joint_velocity.velocity.a1 << " " << command_joint_velocity.velocity.a2 << " " << command_joint_velocity.velocity.a3 << " " << \
		command_joint_velocity.velocity.a4 << " " << command_joint_velocity.velocity.a5 << " " << command_joint_velocity.velocity.a6 << " " << command_joint_velocity.velocity.a7 << " " << state_joint_torque.torque.a1 << " " << state_joint_torque.torque.a2 << " " << state_joint_torque.torque.a3 << " " << \
		state_joint_torque.torque.a4 << " " << state_joint_torque.torque.a5 << " " << state_joint_torque.torque.a6 << " " << state_joint_torque.torque.a7 << "\n";

		loop_rate_->sleep(); 

	}

	data.close();

}; 