
### Steps to run the code
- _Build the package (iiwa-stack) using catkin build (either install orocos-kdl dependancy using 'sudo apt-get install orocos-kdl' or build from source)
- _Orocos-kdl is not needed for if a controller is not used  (e.g. kuka-ddp)
- _Run gazebo launch file by roslaunch iiwa-gazebo iiwa-gazbo.launch

### Gazebo functionality
- _Currently gazebo is in velocity mode (hybrid mode)
- _Example code is provided in 'kuka-iiwa/iiwa_stack_examples/iiwa_tool_examples/src/CommandRobot.cpp


