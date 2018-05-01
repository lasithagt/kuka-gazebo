/**
 * Copyright (C) 2016-2017 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "iiwa_ros.h"


using namespace std;                        

namespace iiwa_ros {
  
  ros::Time last_update_time;
  
 
  void iiwaRosReal::init(ros::NodeHandle& nh)
  {	
    bool isGazebo; // See if it's Gazebo simulation is begin used.
    nh.getParam("isGazebo", isGazebo);


    if (!isGazebo) {
                                                                                         // TODO: put these topics in the parameter file as well. 

      holder_state_pose_.init("iiwa/state/CartesianPose");
      holder_state_joint_position_.init("iiwa/state/JointPosition");
      holder_state_joint_torque_.init("iiwa/state/JointTorque");
      holder_state_wrench_.init("iiwa/state/CartesianWrench");
      holder_state_joint_stiffness_.init("iiwa/state/JointStiffness");
      holder_state_joint_position_velocity_.init("iiwa/state/JointPositionVelocity");
      holder_state_joint_damping_.init("iiwa/state/JointDamping");
      holder_state_joint_velocity_.init("iiwa/state/JointVelocity");
      holder_state_destination_reached_.init("iiwa/state/DestinationReached");
      holder_state_comm_joint_velocity_.init("iiwa/command/JointVelocity");
      
      holder_command_pose_.init("iiwa/command/CartesianPose");
      holder_command_joint_position_.init("iiwa/command/JointPosition");
      holder_command_joint_position_velocity_.init("iiwa/command/JointPositionVelocity");
      holder_command_joint_velocity_.init("iiwa/command/JointVelocity");
      
      smart_servo_service_.setServiceName("iiwa/configuration/configureSmartServo");
      path_parameters_service_.setServiceName("iiwa/configuration/pathParameters");
      time_to_destination_service_.setServiceName("iiwa/state/timeToDestination");
    }
  }
  

  
  bool iiwaRosReal::getCartesianPose(geometry_msgs::PoseStamped& value) {
    return holder_state_pose_.get(value);
  }
  bool iiwaRosReal::getJointPosition(iiwa_msgs::JointPosition& value) {
    return holder_state_joint_position_.get(value);
  }
  bool iiwaRosReal::getJointTorque(iiwa_msgs::JointTorque& value) {
    return holder_state_joint_torque_.get(value);
  }
  bool iiwaRosReal::getJointStiffness(iiwa_msgs::JointStiffness& value) {
    return holder_state_joint_stiffness_.get(value);
  }
  bool iiwaRosReal::getCartesianWrench(geometry_msgs::WrenchStamped& value) {
    return holder_state_wrench_.get(value);
  }
  bool iiwaRosReal::getJointVelocity(iiwa_msgs::JointVelocity& value) {
    return holder_state_joint_velocity_.get(value);
  }
  bool iiwaRosReal::getJointPositionVelocity(iiwa_msgs::JointPositionVelocity& value) {
    return holder_state_joint_position_velocity_.get(value);
  }
  bool iiwaRosReal::getJointDamping(iiwa_msgs::JointDamping& value) {
    return holder_state_joint_damping_.get(value);
  }

  bool iiwaRosReal::getJointCommVelocity(iiwa_msgs::JointVelocity& value) {
    return holder_state_comm_joint_velocity_.get(value);
  }
  
  void iiwaRosReal::setCartesianPose(const geometry_msgs::PoseStamped& position) {
    holder_command_pose_.set(position);
    holder_command_pose_.publishIfNew();
  }
  void iiwaRosReal::setJointPosition(const iiwa_msgs::JointPosition& position)  {
    holder_command_joint_position_.set(position);
    holder_command_joint_position_.publishIfNew();
  }
  void iiwaRosReal::setJointVelocity(const iiwa_msgs::JointVelocity& velocity) {
    holder_command_joint_velocity_.set(velocity);
    holder_command_joint_velocity_.publishIfNew();
    
  }
  void iiwaRosReal::setJointPositionVelocity(const iiwa_msgs::JointPositionVelocity& value) {
    holder_command_joint_position_velocity_.set(value);
    holder_command_joint_position_velocity_.publishIfNew();
  }



 

  void iiwaRosGazebo::init(ros::NodeHandle& nh)
  { 
    bool isGazebo; // See if it's Gazebo simulation is begin used.
    nh.param("isGazebo", isGazebo, true);
    // std::cout << isGazebo <<std::endl;

    if (isGazebo) {
          // TODO: put these topics in the parameter file as well. Pass the hardware interface as from the parameter server
      std::cout << "Initialiing subscribers and publishers..." <<std::endl;
      holder_state.init("/iiwa/joint_states");
      
      // holder_command_joint_position_velocity_.init("iiwa/command/JointPositionVelocity");
      holder_command_joint_1.init("/iiwa/VelocityJointInterface_J1_controller/command");
      holder_command_joint_2.init("/iiwa/VelocityJointInterface_J2_controller/command");
      holder_command_joint_3.init("/iiwa/VelocityJointInterface_J3_controller/command");
      holder_command_joint_4.init("/iiwa/VelocityJointInterface_J4_controller/command");
      holder_command_joint_5.init("/iiwa/VelocityJointInterface_J5_controller/command");
      holder_command_joint_6.init("/iiwa/VelocityJointInterface_J6_controller/command");
      holder_command_joint_7.init("/iiwa/VelocityJointInterface_J7_controller/command");
      ros::Duration(1).sleep();
      
    }
  }
  
  
  bool iiwaRosGazebo::getJointPositionVelocity(iiwa_msgs::JointPositionVelocity& value) {
    sensor_msgs::JointState read;
    holder_state.get(read);
    value.position.a1 = read.position.at(0);
    value.position.a2 = read.position.at(1);
    value.position.a3 = read.position.at(2);
    value.position.a4 = read.position.at(3);
    value.position.a5 = read.position.at(4);
    value.position.a6 = read.position.at(5);
    value.position.a7 = read.position.at(6);


    value.velocity.a1 = read.velocity.at(0);
    value.velocity.a2 = read.velocity.at(1);
    value.velocity.a3 = read.velocity.at(2);
    value.velocity.a4 = read.velocity.at(3);
    value.velocity.a5 = read.velocity.at(4);
    value.velocity.a6 = read.velocity.at(5);
    value.velocity.a7 = read.velocity.at(6);

  }
  
  void iiwaRosGazebo::setJointPosition(const iiwa_msgs::JointPosition& position)  {
    

    std_msgs::Float64 temp;
    // std::cout << "Writing..." <<std::endl;

    temp.data = position.position.a1;
    holder_command_joint_1.set(temp);
    holder_command_joint_1.publishIfNew();

    temp.data = position.position.a2;
    holder_command_joint_2.set(temp);
    holder_command_joint_2.publishIfNew();

    temp.data = position.position.a3;
    holder_command_joint_3.set(temp);
    holder_command_joint_3.publishIfNew();

    temp.data = position.position.a4;
    holder_command_joint_4.set(temp);
    holder_command_joint_4.publishIfNew();

    temp.data = position.position.a5;
    holder_command_joint_5.set(temp);
    holder_command_joint_5.publishIfNew();

    temp.data = position.position.a6;
    holder_command_joint_6.set(temp);
    holder_command_joint_6.publishIfNew();

    temp.data = position.position.a7;
    holder_command_joint_7.set(temp);
    holder_command_joint_7.publishIfNew();
  }
  void iiwaRosGazebo::setJointVelocity(const iiwa_msgs::JointVelocity& velocity) {

    std_msgs::Float64 temp;
    // std::cout << "Writing to Velocity (gazebo mode) ..." <<std::endl;
    temp.data = velocity.velocity.a1;
    holder_command_joint_1.set(temp);
    holder_command_joint_1.publishIfNew();

    temp.data = velocity.velocity.a2;
    holder_command_joint_2.set(temp);
    holder_command_joint_2.publishIfNew();

    temp.data = velocity.velocity.a3;
    holder_command_joint_3.set(temp);
    holder_command_joint_3.publishIfNew();

    temp.data = velocity.velocity.a4;
    holder_command_joint_4.set(temp);
    holder_command_joint_4.publishIfNew();

    temp.data = velocity.velocity.a5;
    holder_command_joint_5.set(temp);
    holder_command_joint_5.publishIfNew();

    temp.data = velocity.velocity.a6;
    holder_command_joint_6.set(temp);
    holder_command_joint_6.publishIfNew();

    temp.data = velocity.velocity.a7;
    holder_command_joint_7.set(temp);
    holder_command_joint_7.publishIfNew();
    
  }

  
  



}
