#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped, Wrench, Vector3

def publishCircleTrajectory():
	rospy.init_node('jointCommandPublisher')
	joint_Publisher = []
	forcex = []
	forcey = []
	forcez = []
	torquex = []
	torquey = []
	torquez = []
	time = []
	joint_Publisher.append(rospy.Publisher('/iiwa/PositionJointInterface_J1_controller/command', Float64, queue_size = 10))
	joint_Publisher.append(rospy.Publisher('/iiwa/PositionJointInterface_J2_controller/command', Float64, queue_size = 10))
	joint_Publisher.append(rospy.Publisher('/iiwa/PositionJointInterface_J3_controller/command', Float64, queue_size = 10))
	joint_Publisher.append(rospy.Publisher('/iiwa/PositionJointInterface_J4_controller/command', Float64, queue_size = 10))
	joint_Publisher.append(rospy.Publisher('/iiwa/PositionJointInterface_J5_controller/command', Float64, queue_size = 10))
	joint_Publisher.append(rospy.Publisher('/iiwa/PositionJointInterface_J6_controller/command', Float64, queue_size = 10))
	joint_Publisher.append(rospy.Publisher('/iiwa/PositionJointInterface_J7_controller/command', Float64, queue_size = 10))

	rospy.Subscriber('/iiwa/ft_sensor_data', WrenchStamped, callback, [forcex,forcey,forcez,torquex,torquey,torquez,time])
	file = open('/home/juan/catkin_kuka/src/kuka-iiwa/iiwa_resources/desired_position_circle.txt','r')
	message = Float64()
	rate_control = rospy.Rate(10)
	print("Starting circle trajectory")
	for line in file:
		jointAngles = line.split()
		jointAngles = jointAngles[1:len(jointAngles)-1]
		count = 0;
		for i in jointAngles:
			message.data = float(i)
			joint_Publisher[count].publish(message)
			count += 1
		rate_control.sleep()
	print("Trajectory completed")
	rospy.spin()

def callback(data, callback_args):
	callback_args[0].append(data.wrench.force.x)
	callback_args[1].append(data.wrench.force.y)
	callback_args[2].append(data.wrench.force.z)
	callback_args[3].append(data.wrench.torque.x)
	callback_args[4].append(data.wrench.torque.y)
	callback_args[5].append(data.wrench.torque.z)
	callback_args[6].append(data.header.stamp)
	print(callback_args[0])

if __name__ == '__main__':
    publishCircleTrajectory()