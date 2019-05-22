cmake_minimum_required(VERSION 2.8.3)
project(scripts_sim)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  # iiwa_ros

)
find_package(orocos_kdl REQUIRED)
find_package(gazebo REQUIRED)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

#add_executable(command_robot src/command_robot.cpp)
#add_dependencies(command_robot iiwa_msgs_generate_messages_cpp)
#target_link_libraries(command_robot ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

#add_executable(save_data src/save_data.cpp)
#add_dependencies(save_data iiwa_msgs_generate_messages_cpp)
#target_link_libraries(save_data ${catkin_LIBRARIES} )

#add_executable(command_robot_moveit src/command_robot_moveit.cpp)
#add_dependencies(command_robot_moveit iiwa_msgs_generate_messages_cpp)
#target_link_libraries(command_robot_moveit ${catkin_LIBRARIES})

#add_executable(setting_control_mode src/setting_control_mode.cpp)
#add_dependencies(setting_control_mode iiwa_msgs_generate_messages_cpp)
#target_link_libraries(setting_control_mode ${catkin_LIBRARIES})


include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${PROJECT_NAME}_gazebo_transport_to_ros_topic ${GAZEBO_LIBRARY_DIRS})
# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#add_executable(listener src/gz_topic_listner.cpp)
#target_link_libraries(listener ${GAZEBO_LIBRARIES} pthread ${catkin_LIBRARIES})

# Compile python bindings for python 2.7
#add_library(pybindings SHARED src/pybindings.cpp)
# Set target specific libraries and includes
#find_package(PythonLibs 2.7 REQUIRED)
#target_compile_options(pybindings PRIVATE -I/usr/local/include/python2.7)
#target_include_directories(pybindings PRIVATE "${PYTHON_INCLUDE_DIRS}")
# Link libraries
#target_link_libraries(pybindings -Wl,--start-group ${catkin_LIBRARIES} -Wl,--end-group)
# Disable 'lib' prefix in front of shared library name
#SET_TARGET_PROPERTIES(pybindings PROPERTIES PREFIX "")

# c++ -O3 -Wall -shared -std=c++11 -fPIC `python2 -m pybind11 --includes` pybindings.cpp -o pybindings`python2-config --extension-suffix`


#catkin_install_python(PROGRAMS src/command_robot.py DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#catkin_python_setup()

