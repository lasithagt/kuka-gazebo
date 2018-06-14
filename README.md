## IIWA STACK
ROS Indigo/Kinetic metapackage for the KUKA LBR IIWA R800/R820 (7/14 Kg).

**Current version : v-1.2.0 for Sunrise 1.11**    
[Using a previous version of Sunrise?](https://github.com/SalvoVirga/iiwa_stack/wiki/FAQ#which-version-of-sunriseossunrise-workbench-is-supported)    

[![Build Status](https://travis-ci.org/SalvoVirga/iiwa_stack.svg?branch=master)](https://travis-ci.org/SalvoVirga/iiwa_stack)
___
### Features
- Native ROSJava nodes running on the robot as a Sunrise RoboticApplication: supports ROS parameters, topics, services, etc.
- Integration of KUKA's SmartServo motions:
  - joint position, joint velocity and cartesian position control via simple ROS messages. 
  - online configuration of JointImpedance, CartesianImpedance, DesiredForce and Sine(Force)Pattern via ROS service.
  - online configuration of joint velocity and acceleration via ROS service.
  - updates on the time left to reach the commanded destination via ROS service.
- The Sunrise tool to use can be selected via a ROS parameter.
- Gravity compensation by setting the appropriate JointImpedance values.
- NTP synchronization with a server running on the ROS master
- full MoveIt! integration
- Gazebo support

___
### Usage
__The features and usage of the stack are described in depth on its  [WIKI][8].__  
We **_strongly_** suggest to have a look at the wiki to have a better understanding of the code, both for its use and its extension.     
Do you have problems? First, please check the [**FAQs**](https://github.com/SalvoVirga/iiwa_stack/wiki/FAQ). Issues or emails are always welcome!

___
### Citation

If you use iiwa_stack for reseach, you could cite the following work. It is the first publication where it was used.

    @article{hennersperger2016towards,
        title={Towards MRI-based Autonomous Robotic US Acquisitions: A First Feasibility Study},
        author={Hennersperger, Christoph and Fuerst, Bernhard and Virga, Salvatore and Zettinig, Oliver and Frisch, Benjamin and Neff, Thomas and Navab, Nassir},
        journal={IEEE transactions on medical imaging},
        year={2016},
        publisher={IEEE}
    }

___
### Acknowledgements
This repository takes inspiration from the work of :
- _Centro E. Piaggio_ and their [ROS interface for the KUKA LBR 4+][1]
- _Mohammad Khansari_ and his [IIWA-ROS communication inteface][2] 
- _Robert Krug_ and his [IIWA URDF and Gazebo package][7]      

Most of the original files were completely refactored though.


=======
# kuka-iiwa
This includes all the plugins required to run the KUKA as well the Gazebo. A fork from iiwa-stack. Customized and added some functions


### Steps to run the code
- Build the package (iiwa-stack) using catkin build (either install orocos-kdl dependancy using 'sudo apt-get install orocos-kdl' or build from source)
- Launch script is located at kuka-iiwa/iiwa-gazebo/launch/launch_script.sh
- Run launch scipt by bash ./launch_script.sh
- Press a key to load (on the terminal) the controllers when Gazebo environment is loaded.

### Gazebo functionality
- Currently gazebo is in velocity mode (hybrid mode)
- Example code is provided in 'kuka-iiwa/iiwa_stack_examples/iiwa_tool_examples/src/CommandRobot.cpp



