#!/usr/bin/python
import rospy
import math
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
			time = rospy.get_rostime()
			startTime = time.nsecs
			currentTime = startTime
			while((currentTime-startTime)*0.000000001<float(userinput[2])):
				currentTime = rospy.get_rostime().nsecs*0.000000001
				pos = float(userinput[0])+float(userinput[1])*math.sin(0.000000001*(currentTime-startTime))
				message.data=pos
				surfacePublisher.publish(message)
				rospy.Rate(10)
		elif (command == 'Step response surface'):
			position = raw_input('Input step response set point\n')
			message.data = float(position)
			surfacePublisher.publish(message)


if __name__ == '__main__':
	closeGripper()
