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

#pragma once

#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointStiffness.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointPositionVelocity.h>
#include <iiwa_msgs/JointDamping.h>
#include <std_msgs//Time.h>
#include <std_msgs//Float64.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <vector>

#include <string>
#include <mutex>

namespace iiwa_ros {
  
  extern ros::Time last_update_time;

  template <typename ROSMSG>
  class iiwaHolder {
  public:
    iiwaHolder() : is_new(false) {}
    
    void set_value(const ROSMSG& value) {
      mutex.lock();
      data = value;
      is_new = true;
      mutex.unlock();
    }
    
    bool get_value(ROSMSG& value) {
      bool was_new = false;
      mutex.lock();
      value = data;
      was_new = is_new;
      is_new = false;
      mutex.unlock();
      
      return was_new;
    }
    
    bool has_new_value() {
      return is_new;
    }
    
    ROSMSG get_value_unsynchronized() {
      return data;
    }
    
  private:
    ROSMSG data;
    bool is_new;
    std::mutex mutex;
  };
  

  template <typename ROSMSG>
  class iiwaStateHolder {
  public:
    void init(const std::string& topic) {
      ros::NodeHandle nh;
      subscriber = nh.subscribe<ROSMSG>(topic, 1, &iiwaStateHolder<ROSMSG>::set, this);
    }
    
    bool has_new_value() {
      return holder.has_new_value();
    }
    
    void set(ROSMSG value) {
      last_update_time = ros::Time::now();
      holder.set_value(value);
    }
    
    bool get(ROSMSG& value) {
      return holder.get_value(value);        
    }
  private:
    iiwaHolder<ROSMSG> holder;
    ros::Subscriber subscriber;
  };
  
  
  template <typename ROSMSG>
  class iiwaCommandHolder {
  public:
    void init(const std::string& topic) {
      ros::NodeHandle nh;
      publisher = nh.advertise<ROSMSG>(topic, 1);
      // std::cout << "Init Publishing..." <<std::endl;
    }
    
    void set(const ROSMSG& value) {
      holder.set_value(value);
    }
    
    ROSMSG get() {
      return holder.get_value_unsynchronized();
    }
    
    void publishIfNew() {
      static ROSMSG msg;
      if (publisher.getNumSubscribers() && holder.get_value(msg))
        // std::cout << "Publishing..." <<std::endl;
        publisher.publish(msg);
    }
  private:
    ros::Publisher publisher;
    iiwaHolder<ROSMSG> holder;
  };
   



  // Abstract class for iiwaROS, The messages for Gazebo and real time system are different.
  class iiwaRos {
  public:
    

    // iiwaRos();
    
 
    virtual void init(ros::NodeHandle& nh)=0;
    
 
    virtual bool getCartesianPose(geometry_msgs::PoseStamped& value) {return 0;}
    

    virtual bool getJointPosition(iiwa_msgs::JointPosition& value) {return 0;}
   
    virtual bool getJointTorque(iiwa_msgs::JointTorque& value) {return 0;}
    

    virtual bool getJointStiffness(iiwa_msgs::JointStiffness& value) {return 0;}
    
   
    virtual bool getCartesianWrench(geometry_msgs::WrenchStamped& value) {return 0;}
    

    virtual bool getJointVelocity(iiwa_msgs::JointVelocity& value) {return 0;}
    


    virtual bool getJointCommVelocity(iiwa_msgs::JointVelocity& value) {return 0;}


    virtual bool getJointPositionVelocity(iiwa_msgs::JointPositionVelocity& value) {return 0;}
    

    virtual bool getJointDamping(iiwa_msgs::JointDamping& value) {return 0;}
    

    virtual void setCartesianPose(const geometry_msgs::PoseStamped& position) {}
    

    virtual void setJointPosition(const iiwa_msgs::JointPosition& position) {};
    

    virtual void setJointVelocity(const iiwa_msgs::JointVelocity& velocity) {};
    

    virtual void setJointPositionVelocity(const iiwa_msgs::JointPositionVelocity& value) {};
    
 
    bool getRobotIsConnected() {
      ros::Duration diff = (ros::Time::now() - last_update_time);
      return (diff < ros::Duration(0.25));
    }
    
  protected:
    iiwaStateHolder<geometry_msgs::PoseStamped> holder_state_pose_;
    iiwaStateHolder<iiwa_msgs::JointPosition> holder_state_joint_position_;
    iiwaStateHolder<iiwa_msgs::JointTorque> holder_state_joint_torque_;
    iiwaStateHolder<geometry_msgs::WrenchStamped> holder_state_wrench_;
    iiwaStateHolder<iiwa_msgs::JointDamping> holder_state_joint_damping_;
    iiwaStateHolder<iiwa_msgs::JointStiffness> holder_state_joint_stiffness_;
    iiwaStateHolder<iiwa_msgs::JointVelocity> holder_state_joint_velocity_;
    iiwaStateHolder<iiwa_msgs::JointVelocity> holder_state_comm_joint_velocity_;
    iiwaStateHolder<iiwa_msgs::JointPositionVelocity> holder_state_joint_position_velocity_;
    iiwaStateHolder<std_msgs::Time> holder_state_destination_reached_;
    
    iiwaCommandHolder<geometry_msgs::PoseStamped> holder_command_pose_;
    iiwaCommandHolder<iiwa_msgs::JointPosition> holder_command_joint_position_;
    iiwaCommandHolder<iiwa_msgs::JointVelocity> holder_command_joint_velocity_;
    iiwaCommandHolder<iiwa_msgs::JointPositionVelocity> holder_command_joint_position_velocity_;
    
    SmartServoService smart_servo_service_;
    PathParametersService path_parameters_service_;
    TimeToDestinationService time_to_destination_service_;
  };

  class iiwaRosReal: public iiwaRos {
  public:

    void init(ros::NodeHandle& nh);
    bool getCartesianPose(geometry_msgs::PoseStamped& value);
    bool getJointPosition(iiwa_msgs::JointPosition& value); 
    bool getJointTorque(iiwa_msgs::JointTorque& value); 
    bool getJointStiffness(iiwa_msgs::JointStiffness& value); 
    bool getCartesianWrench(geometry_msgs::WrenchStamped& value);
    bool getJointVelocity(iiwa_msgs::JointVelocity& value);
    bool getJointPositionVelocity(iiwa_msgs::JointPositionVelocity& value);
    bool getJointDamping(iiwa_msgs::JointDamping& value); 
    bool getJointCommVelocity(iiwa_msgs::JointVelocity& value); 
    void setCartesianPose(const geometry_msgs::PoseStamped& position);
    void setJointPosition(const iiwa_msgs::JointPosition& position); 
    void setJointVelocity(const iiwa_msgs::JointVelocity& velocity); 
    void setJointPositionVelocity(const iiwa_msgs::JointPositionVelocity& value); 
  };



  class iiwaRosGazebo: public iiwaRos {

  public:
    void init(ros::NodeHandle& nh);
    bool getJointPositionVelocity(iiwa_msgs::JointPositionVelocity& value);
    void setJointPosition(const iiwa_msgs::JointPosition& position);
    void setJointVelocity(const iiwa_msgs::JointVelocity& velocity);

    ros::NodeHandle initPy(double rate) {
        int argc = 1;
        char** argv = new char *[1];
        ros::init(argc, argv, "CommandRobot");
        ros::NodeHandle nh("~");
        bool isGazebo = true;
        double ros_rate;
        nh.getParam("isGazebo", isGazebo);
        nh.param("ros_rate", ros_rate, rate);
        return nh;
    }

    void setJointVelocityPy(std::vector<double> velocity=(std::vector<double>){0, 0, 0, 0, 0, 0, 0}) {
        iiwa_msgs::JointVelocity command_joint_velocity;

        command_joint_velocity.velocity.a1 = velocity[0];
        command_joint_velocity.velocity.a2 = velocity[1];
        command_joint_velocity.velocity.a3 = velocity[2];
        command_joint_velocity.velocity.a4 = velocity[3];
        command_joint_velocity.velocity.a5 = velocity[4];
        command_joint_velocity.velocity.a6 = velocity[5];
        command_joint_velocity.velocity.a7 = velocity[6];

        setJointVelocity(command_joint_velocity);
    }

  private:
    iiwaStateHolder<sensor_msgs::JointState> holder_state;
    iiwaCommandHolder<std_msgs::Float64> holder_command_joint_1;
    iiwaCommandHolder<std_msgs::Float64> holder_command_joint_2;
    iiwaCommandHolder<std_msgs::Float64> holder_command_joint_3;
    iiwaCommandHolder<std_msgs::Float64> holder_command_joint_4;
    iiwaCommandHolder<std_msgs::Float64> holder_command_joint_5;
    iiwaCommandHolder<std_msgs::Float64> holder_command_joint_6;
    iiwaCommandHolder<std_msgs::Float64> holder_command_joint_7;
  };




  
  
}
