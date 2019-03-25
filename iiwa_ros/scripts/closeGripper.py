#!/usr/bin/python
import rospy
import math
import numpy
from std_msgs.msg import Float64

def closeGripper():
	rospy.init_node('GripperCommandPublisher')
	clawPublishers = [rospy.Publisher('/iiwa/EffortJointInterface_J8_controller/command', Float64),rospy.Publisher('/iiwa/EffortJointInterface_J9_controller/command', Float64),rospy.Publisher('/iiwa/EffortJointInterface_J10_controller/command', Float64),rospy.Publisher('/iiwa/EffortJointInterface_J11_controller/command', Float64)]
	surfacePublisher = rospy.Publisher('/iiwa/EffortJointInterface_PS_controller/command', Float64)
	while not rospy.is_shutdown():
		command = raw_input('Specify command\n')
		message = Float64()
		if (command == 'Engage gripper'):
			torque = raw_input('Specify torque\n')
			message.data = float(torque)
			for publisher in clawPublishers:
				publisher.publish(message.data)
		elif (command == 'Disengage gripper'):
			message = Float64()
			message.data = -0.01
			for publisher in clawPublishers:
				publisher.publish(message)
		elif (command == 'Oscillate surface'):
			userinput = raw_input('Specify amplitude, offset and duration in s\n')
			userinput = userinput.split()
			time = numpy.linspace(0,float(userinput[2]),10*float(userinput[2]))
			rate = rospy.Rate(10)
			for t in time:
				pos = float(userinput[1])+float(userinput[0])*math.sin(math.pi*t)
				message.data=pos
				surfacePublisher.publish(message)
				rate.sleep()
		elif (command == 'Step response surface'):
			position = raw_input('Input step response set point\n')
			message.data = float(position)
			surfacePublisher.publish(message)
		else:
			print("Command not recognized")


if __name__ == '__main__':
	closeGripper()
