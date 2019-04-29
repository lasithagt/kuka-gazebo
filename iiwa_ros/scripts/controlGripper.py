#!/usr/bin/python
import rospy
import math
import numpy
from std_msgs.msg import Float64


def gripObject():
    rospy.init_node('gripperEngager')
    message = Float64()
    gripper1Publishers = [rospy.Publisher('/iiwa/EffortJointInterface_claw_1_joint_controller/command',
                                          Float64), rospy.Publisher('/iiwa/EffortJointInterface_claw_2_joint_controller/command', Float64)]
    gripper2Publishers = [rospy.Publisher('/iiwa/EffortJointInterface_2_claw_1_joint_controller/command',
                                          Float64), rospy.Publisher('/iiwa/EffortJointInterface_2_claw_2_joint_controller/command', Float64)]
    liftingJointPublisher = rospy.Publisher('/iiwa/EffortJointInterface_J_1_controller/command', Float64)
    while not rospy.is_shutdown():
        command = raw_input('Engage, Disengage gripper or Lift part')
        if (command == 'Engage gripper'):
                torque = raw_input('Specify torque\n')
                message.data = float(torque)
                for publisher in gripper1Publishers:
                    publisher.publish(message.data)
                for publisher in gripper2Publishers:
                    publisher.publish(message.data)
        elif (command == 'Disengage gripper'):
            message = Float64()
            message.data = -0.01
            for publisher in gripper1Publishers:
                publisher.publish(message.data)
            for publisher in gripper2Publishers:
                publisher.publish(message.data)
        elif (command == 'Lift part'):
            message.data = -0.4
            liftingJointPublisher.publish(message.data)

if __name__ == '__main__':
    gripObject()
