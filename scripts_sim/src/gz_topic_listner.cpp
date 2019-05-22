/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/WrenchStamped.h>

ros::Publisher pub;

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void cb(const ConstWrenchStampedPtr &_msg)
{
  // Dump the message contents to stdout.
  // std::cout << _msg->wrench().force().x();
  // std::cout << "\n";

  geometry_msgs::WrenchStamped msgWrenchedStamped;
  // try WrenchStamped msgWrenchedStamped;
  msgWrenchedStamped.header.stamp = ros::Time::now();
  msgWrenchedStamped.wrench.force.x = _msg->wrench().force().x();
  msgWrenchedStamped.wrench.force.y = _msg->wrench().force().y();
  msgWrenchedStamped.wrench.force.z = _msg->wrench().force().z();
  msgWrenchedStamped.wrench.torque.x = _msg->wrench().torque().x();
  msgWrenchedStamped.wrench.torque.y = _msg->wrench().torque().y();
  msgWrenchedStamped.wrench.torque.z = _msg->wrench().torque().z();
  pub.publish(msgWrenchedStamped);
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Load ROS
  const std::string node_name = "gazebo_to_ros";
  ros::init(_argc, _argv, node_name);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Create ROS node and init
  ros::NodeHandle n;

  // Create the publisher.
  pub = n.advertise<geometry_msgs::WrenchStamped>("gazeboToROS", 100);

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/iiwa14/frame_kuka/wrench", cb);

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}