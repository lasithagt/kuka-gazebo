#!/usr/bin/python
import rospy
import math
import numpy
from std_msgs.msg import Float64
from trac_ik_python.trac_ik import IK


def gripObject():
    rospy.init_node('ManipulatorCommandPublisher')
    arm1Publishers = [rospy.Publisher('/iiwa/EffortJointInterface_J_1_controller/command', Float64), rospy.Publisher('/iiwa/EffortJointInterface_J_2_controller/command', Float64),
                      rospy.Publisher('/iiwa/EffortJointInterface_J_3_controller/command', Float64), rospy.Publisher('/iiwa/EffortJointInterface_J_4_controller/command', Float64), rospy.Publisher('/iiwa/EffortJointInterface_gripper_base_joint_controller/command', Float64)]
    arm2Publishers = [rospy.Publisher('/iiwa/EffortJointInterface_J_21_controller/command', Float64), rospy.Publisher('/iiwa/EffortJointInterface_J_22_controller/command', Float64),
                      rospy.Publisher('/iiwa/EffortJointInterface_J_23_controller/command', Float64), rospy.Publisher('/iiwa/EffortJointInterface_J_24_controller/command', Float64), rospy.Publisher('/iiwa/EffortJointInterface_2_gripper_base_joint_controller/command', Float64)]
    surfacePublisher = rospy.Publisher(
        '/iiwa/EffortJointInterface_PS_controller/command', Float64)
    gripper1Publishers = [rospy.Publisher('/iiwa/EffortJointInterface_claw_1_joint_controller/command',
                                          Float64), rospy.Publisher('/iiwa/EffortJointInterface_claw_2_joint_controller/command', Float64)]
    gripper2Publishers = [rospy.Publisher('/iiwa/EffortJointInterface_2_claw_1_joint_controller/command',
                                          Float64), rospy.Publisher('/iiwa/EffortJointInterface_2_claw_2_joint_controller/command', Float64)]
    ik_solver = IK("base_link", "gripper_base_link")
    ik_solver2 = IK("base_link", "2_gripper_base_link")

    print("IK solver uses link chain:")
    print(ik_solver.link_names)

    print("IK solver 2 uses link chain:")
    print(ik_solver2.link_names)

    print("IK solver base frame:")
    print(ik_solver.base_link)

    print("IK solver 2 base frame:")
    print(ik_solver2.base_link)

    print("IK solver tip link:")
    print(ik_solver.tip_link)

    print("IK solver 2 tip link:")
    print(ik_solver2.tip_link)

    print("IK solver for joints:")
    print(ik_solver.joint_names)

    print("IK solver 2 for joints:")
    print(ik_solver2.joint_names)

    lb, up = ik_solver.get_joint_limits()
    print("Lower bound: " + str(lb))
    print("Upper bound: " + str(up))

    lb, up = ik_solver2.get_joint_limits()
    print("Lower bound2: " + str(lb))
    print("Upper bound2: " + str(up))

    qinit = [-0.2934661551582609, -0.08162442352922827, -
             0.003015193424318774, 0.40880813669625127, -0.04521353165214581]
    qinit2 = [-0.03967407282925528, -0.1412745962124653,
              0.0805731133513401, 0.587582286146036, -0.12278869200741127]

    brx = bry = 90
    brz = 90
    bx = by = bz = 0.02

    while not rospy.is_shutdown():
        message = Float64()
        command = raw_input('Specify arm')
        if int(command) == 1:
            command = raw_input('Specify location X\n')
            xyz = [float(command)]
            command = raw_input('Specify location Y\n')
            xyz.append(float(command))
            command = raw_input('Specify location Z\n')
            xyz.append(float(command))
            command = raw_input('Specify location Rx\n')
            rxyz = [float(command)]
            command = raw_input('Specify location Ry\n')
            rxyz.append(float(command))
            command = raw_input('Specify location Rz\n')
            rxyz.append(float(command))
            command = raw_input('Specify location Ry\n')
            rxyz.append(float(command))
            sol = ik_solver.get_ik(qinit, xyz[0], xyz[1], xyz[2], rxyz[0], rxyz[1], rxyz[2], rxyz[3],
                                   bx, by, bz,
                                   brx, bry, brz)
            print(sol)
            if sol is not None:
                print "X, Y, Z: " + str((xyz[0], xyz[1], xyz[2]))
                print "SOL: " + str(sol)
                command = raw_input("Go to coordinates? y/n")
                if command == "y":
                    qinit = sol
                    count = 0
                    for c in sol:
                        message.data = c
                        arm1Publishers[count].publish(message)
                        count = count + 1
        elif int(command) == 2:
            command = raw_input('Specify location X\n')
            xyz = [float(command)]
            command = raw_input('Specify location Y\n')
            xyz.append(float(command))
            command = raw_input('Specify location Z\n')
            xyz.append(float(command))
            command = raw_input('Specify location Rx\n')
            rxyz = [float(command)]
            command = raw_input('Specify location Ry\n')
            rxyz.append(float(command))
            command = raw_input('Specify location Rz\n')
            rxyz.append(float(command))
            command = raw_input('Specify location Ry\n')
            rxyz.append(float(command))
            sol = ik_solver2.get_ik(qinit2, xyz[0], xyz[1], xyz[2], rxyz[0], rxyz[1], rxyz[2], rxyz[3],
                                    bx, by, bz,
                                    brx, bry, brz)
            print(sol)
            if sol is not None:
                print "X, Y, Z: " + str((xyz[0], xyz[1], xyz[2]))
                print "SOL: " + str(sol)
                command = raw_input("Go to coordinates? y/n")
                if command == "y":
                    qinit2 = sol
                    count = 0
                    for c in sol:
                        message.data = c
                        arm2Publishers[count].publish(message)
                        count = count + 1
    # elif int(command) == 2:
    #   command = raw_input('Gripper number? 1/2')
    #   torque = float(raw_input('Specify torque'))
    #   if int(command) == 1:
    #           for i in gripper1Publishers:
    #               message.data = torque
    #               i.publish(message)
    #   elif int(command) == 2:
    #       for i in gripper2Publishers:
    #               message.data = torque
    #               i.publish(message)

    #   if (command == 'Engage gripper'):
    #       torque = raw_input('Specify torque\n')
    #       message.data = float(torque)
    #       for publisher in clawPublishers:
    #           publisher.publish(message.data)
    #   elif (command == 'Disengage gripper'):
    #       message = Float64()
    #       message.data = -0.01
    #       for publisher in clawPublishers:
    #           publisher.publish(message)
    #   elif (command == 'Oscillate surface'):
    #       userinput = raw_input('Specify amplitude, offset and duration in s\n')
    #       userinput = userinput.split()
    #       time = numpy.linspace(0,float(userinput[2]),10*float(userinput[2]))
    #       rate = rospy.Rate(10)
    #       for t in time:
    #           pos = float(userinput[1])+float(userinput[0])*math.sin(math.pi*t)
    #           message.data=pos
    #           surfacePublisher.publish(message)
    #           rate.sleep()
    #   elif (command == 'Step response surface'):
    #       position = raw_input('Input step response set point\n')
    #       message.data = float(position)
    #       surfacePublisher.publish(message)
    #   else:
    #       print("Command not recognized")


if __name__ == '__main__':
    gripObject()
