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
#include <kdl/frames.hpp>



// getTimeToDestination() can also return negative values and the info from the cabinet take some milliseconds to update once the motion is started.
// That means that if you call getTimeToDestination() right after you set a target pose, you might get the wrong info (e.g. a negative number).
// This function tried to call getTimeToDestination() until something meaningful is obtained or until a maximum amount of time passed.
void sleepForMotion(iiwa_ros::iiwaRos& iiwa, const double maxSleepTime) {
    double ttd = iiwa.getTimeToDestinationService().getTimeToDestination();
    ros::Time start_wait = ros::Time::now();
    while (ttd < 0.0 && (ros::Time::now() - start_wait) < ros::Duration(maxSleepTime)) {
        ros::Duration(0.5).sleep();
        ttd = iiwa.getTimeToDestinationService().getTimeToDestination();
    }
    if (ttd > 0.0) {
        ROS_INFO_STREAM("Sleeping for " << ttd << " seconds.");
        ros::Duration(ttd).sleep();
    }
}

int readFile_nlines(const std::string fileNm) {
    int n_data = 0;
    int x;
    std::ifstream inFile;
    std::string str;


    inFile.open(fileNm);
    if (!inFile) {
        std::cout << "Unable to open file";
        exit(1); // terminate with error
    }

    while (std::getline(inFile, str)) {
        n_data += 1;
    }

    inFile.close();

    return n_data;
}


void readFile(Eigen::MatrixXf &mat, const std::string fileNm) {
    int n_data = 0;
    int x;
    std::ifstream inFile;
    std::string str;


    inFile.open(fileNm);
    if (!inFile) {
        std::cout << "Unable to open file";
        exit(1); // terminate with error
    }

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

    boost::char_separator<char> sep(" "); //Use space as the delimiter
    while (std::getline(inFile, str)) {
        std::vector<float> line;
        tokenizer tok(str, sep);
        //
        std::transform( tok.begin(), tok.end(), std::back_inserter(line),
                &boost::lexical_cast<float,std::string> );
        int t = 0;
        for(std::vector<float>::iterator it = line.begin(); it != line.end(); ++it) {
            mat(t,n_data) = *it;
            // std::cout << line.size() << std::endl;
            t += 1;
        }

        n_data += 1;
        // std::copy(line.begin(), line.end(), std::ostream_iterator<float>(std::cout,"\n") ); //Print those for testing
    }

    inFile.close();
    // std::cout << "Sum = " << n_data << std::endl;

}


int main (int argc, char **argv) {

    // Initialize ROS
    ros::init(argc, argv, "CommandRobot");
    ros::NodeHandle nh("~");

    bool isGazebo = true;
    double ros_rate;

    // get parameters
    nh.getParam("isGazebo", isGazebo);
    nh.param("ros_rate", ros_rate, 60.00);
    // nh.param("isGazebo", isGazebo, true);

    // ROS spinner.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    iiwa_ros::iiwaRosGazebo my_iiwa;
    my_iiwa.init(nh);



    // Dynamic parameter to choose the rate at wich this node should run

    // nh.param("ros_rate", ros_rate, 100.0); // 0.1 Hz = 10 seconds

    ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

    geometry_msgs::PoseStamped command_cartesian_position;
    iiwa_msgs::JointPosition command_joint_position;
    iiwa_msgs::JointVelocity command_joint_velocity;

    // States to measure
    iiwa_msgs::JointPosition state_joint_position;
    iiwa_msgs::JointPositionVelocity state_joint_position_velocity;
    iiwa_msgs::JointVelocity state_joint_velocity;
    iiwa_msgs::JointTorque state_joint_torque;

    // bool new_pose = false, motion_done = false;

    // int direction = 1;

    int n_data_qd, n_data_q;

    n_data_qd = readFile_nlines(getenv("HOME") + std::string("/catkin_ws/src/kuka-iiwa/iiwa_tool_examples/src/data/desired_velocity.txt"));
    n_data_q = readFile_nlines(getenv("HOME") + std::string("/catkin_ws/src/kuka-iiwa/iiwa_tool_examples/src/data/desired_position.txt"));

    std::cout << "N = " << n_data_q << std::endl;
    Eigen::MatrixXf data_qd(7, n_data_qd);
    Eigen::MatrixXf data_q(7, n_data_q);

    // States to measure in the matrix
    Eigen::MatrixXf data_state_velocity(7,n_data_q);
    Eigen::MatrixXf data_state_position(7,n_data_q);
    Eigen::MatrixXf data_state_torque(7,n_data_q);

    readFile(data_qd, getenv("HOME") + std::string("/catkin_ws/src/kuka-iiwa/iiwa_tool_examples/src/data/desired_velocity.txt"));
    readFile(data_q, getenv("HOME") + std::string("/catkin_ws/src/kuka-iiwa/iiwa_tool_examples/src/data/desired_position.txt"));

    std::ofstream data;
    data.open ("JointData.txt");

    int init = 0;

    while (ros::ok()) {
        if (1) {
            /*my_iiwa.getSmartServoService().setJointImpedanceMode(iiwa_ros::jointQuantityFromDouble(5000,5000,5500,5000,5500,5000,5500), iiwa_ros::jointQuantityFromDouble(0.7));
              if (!init) {
              while (!my_iiwa.getJointPosition(command_joint_position)) {} //If the destination has already been achieved. idle.
              command_joint_position.position.a1 = data_q(0,0);
              command_joint_position.position.a2 = data_q(1,0);
              command_joint_position.position.a3 = data_q(2,0);
              command_joint_position.position.a4 = data_q(3,0);
              command_joint_position.position.a5 = data_q(4,0);
              command_joint_position.position.a6 = data_q(5,0);
              command_joint_position.position.a7 = data_q(6,0);

              my_iiwa.setJointPosition(command_joint_position);
              std::cerr << "Sent Command" << std::endl;
              std::cerr << command_joint_position << std::endl;

              sleepForMotion(my_iiwa, 5.0);

            //loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.
            init = 1;

            }*/

            for (int j = 0;j < n_data_qd;j++) {


                //my_iiwa.getJointPosition(state_joint_position);
                //my_iiwa.getJointVelocity(state_joint_velocity);
                //my_iiwa.getJointTorque(state_joint_torque);


                //data << state_joint_position.position.a1 << " " << state_joint_position.position.a2 << " " << state_joint_position.position.a3 << " " << \
                //state_joint_position.position.a4 << " " << state_joint_position.position.a5 << " " << state_joint_position.position.a6 << " " << state_joint_position.position.a7 << state_joint_velocity.velocity.a1 << " " << state_joint_velocity.velocity.a2 << " " << state_joint_velocity.velocity.a3 << " " << \
                //state_joint_velocity.velocity.a4 << " " << state_joint_velocity.velocity.a5 << " " << state_joint_velocity.velocity.a6 << " " << state_joint_velocity.velocity.a7 << state_joint_torque.torque.a1 << " " << state_joint_torque.torque.a2 << " " << state_joint_torque.torque.a3 << " " << \
                //state_joint_torque.torque.a4 << " " << state_joint_torque.torque.a5 << " " << state_joint_torque.torque.a6 << " " << state_joint_torque.torque.a7 << "\n";

                double scale;
                scale = 10;
                my_iiwa.getJointPositionVelocity(state_joint_position_velocity);
                command_joint_velocity.velocity.a1 = data_qd(0,j)/scale;
                command_joint_velocity.velocity.a2 = data_qd(1,j)/scale;
                command_joint_velocity.velocity.a3 = data_qd(2,j)/scale;
                command_joint_velocity.velocity.a4 = data_qd(3,j)/scale;
                command_joint_velocity.velocity.a5 = data_qd(4,j)/scale;
                command_joint_velocity.velocity.a6 = data_qd(5,j)/scale;
                command_joint_velocity.velocity.a7 = data_qd(6,j)/scale;

                // std::cerr << command_joint_velocity << std::endl;

                my_iiwa.setJointVelocity(command_joint_velocity);




                //std::cerr << "Sending velocity commands" << std::endl;
                loop_rate_->sleep();
            }
            data.close();
            ros::shutdown();
        }

        else {
            ROS_WARN_STREAM("Robot is not connected...");
            ros::Duration(5.0).sleep(); // 5 seconds
        }
    }


};
