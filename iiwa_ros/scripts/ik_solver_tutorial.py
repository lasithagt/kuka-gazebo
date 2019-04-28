#!/usr/bin/python

from trac_ik_python.trac_ik import IK
from numpy.random import random
import time
import rospy
import math
import numpy
from std_msgs.msg import Float64


if __name__ == '__main__':
    # roslaunch pr2_description upload_pr2.launch
    # Needed beforehand
    rospy.init_node('ManipulatorCommandPublisher')
    arm1Publishers = [rospy.Publisher('/iiwa/EffortJointInterface_dual_arm_base_joint_controller', Float64), rospy.Publisher('/iiwa/EffortJointInterface_J_1_controller/command', Float64), rospy.Publisher('/iiwa/EffortJointInterface_J_2_controller/command', Float64),
                      rospy.Publisher('/iiwa/EffortJointInterface_J_3_controller/command', Float64), rospy.Publisher('/iiwa/EffortJointInterface_J_4_controller/command', Float64), rospy.Publisher('/iiwa/EffortJointInterface_gripper_base_joint_controller/command', Float64)]
    message = Float64()
    ik_solver = IK("iiwa_link_7", "gripper_base_link")

    print("IK solver uses link chain:")
    print(ik_solver.link_names)

    print("IK solver base frame:")
    print(ik_solver.base_link)

    print("IK solver tip link:")
    print(ik_solver.tip_link)

    print("IK solver for joints:")
    print(ik_solver.joint_names)

    print("IK solver using joint limits:")
    lb, up = ik_solver.get_joint_limits()
    print("Lower bound: " + str(lb))
    print("Upper bound: " + str(up))

    qinit = [0.] * ik_solver.number_of_joints
    x = y = z = 0.0
    rx = ry = rz = 0.0
    rw = 1.0
    bx = by = bz = 0.001
    brx = bry = brz = 0.1

    # Generate a set of random coords in the arm workarea approx
    NUM_COORDS = 200
    rand_coords = []
    for _ in range(NUM_COORDS):
        x = random() * 0.5
        y = random() * 0.6 + -0.3
        z = random() * 0.7 + -0.35
        rand_coords.append((x, y, z))

    # Check some random coords with fixed orientation
    avg_time = 0.0
    num_solutions_found = 0
    for x, y, z in rand_coords:
        ini_t = time.time()
        sol = ik_solver.get_ik(qinit,
                               x, y, z,
                               rx, ry, rz, rw,
                               bx, by, bz,
                               brx, bry, brz)
        fin_t = time.time()
        call_time = fin_t - ini_t
        # print "IK call took: " + str(call_time)
        avg_time += call_time
        if sol:
            print "X, Y, Z: " + str((x, y, z))
            print "SOL: " + str(sol)
            num_solutions_found += 1
    avg_time = avg_time / NUM_COORDS

    print
    print "Found " + str(num_solutions_found) + " of 200 random coords"
    print "Average IK call time: " + str(avg_time)
    print

    # Check if orientation bounds work
    avg_time = 0.0
    num_solutions_found = 0
    brx = bry = brz = 9999.0  # We don't care about orientation
    for x, y, z in rand_coords:
        ini_t = time.time()
        sol = ik_solver.get_ik(qinit,
                               x, y, z,
                               rx, ry, rz, rw,
                               bx, by, bz,
                               brx, bry, brz)
        fin_t = time.time()
        call_time = fin_t - ini_t
        # print "IK call took: " + str(call_time)
        avg_time += call_time
        if sol:
            print "X, Y, Z: " + str((x, y, z))
            print "SOL: " + str(sol)
            num_solutions_found += 1
            command = raw_input("Go to coordinates? y/n")
            if command == "y":
                mylist = numpy.linspace(1,10,10)
                count = 0
                tempsol = list(sol)
                for i in tempsol:
                    tempsol[count] = i/10
                    count = count+1
                rate = rospy.Rate(2)
                for i in mylist:
                    count = 0;
                    for c in tempsol:
                        message.data = c*i
                        arm1Publishers[count].publish(message)
                        count = count+1;
                    rate.sleep()



    avg_time = avg_time / NUM_COORDS
    print
    print "Found " + str(num_solutions_found) + " of 200 random coords ignoring orientation"
    print "Average IK call time: " + str(avg_time)
    print

    # Check if big position and orientation bounds work
    avg_time = 0.0
    num_solutions_found = 0
    bx = by = bz = 9999.0
    brx = bry = brz = 9999.0
    for x, y, z in rand_coords:
        ini_t = time.time()
        sol = ik_solver.get_ik(qinit,
                               x, y, z,
                               rx, ry, rz, rw,
                               bx, by, bz,
                               brx, bry, brz)
        fin_t = time.time()
        call_time = fin_t - ini_t
        # print "IK call took: " + str(call_time)
        avg_time += call_time
        if sol:
            # print "X, Y, Z: " + str( (x, y, z) )
            # print "SOL: " + str(sol)
            num_solutions_found += 1

    avg_time = avg_time / NUM_COORDS

    print
    print "Found " + str(num_solutions_found) + " of 200 random coords ignoring everything"
    print "Average IK call time: " + str(avg_time)
    print

# std::vector<double> CartToJnt(const std::vector<double> q_init,
# const double x, const double y, const double z,
# const double rx, const double ry, const double rz, const double rw,
# // bounds x y z
# const double boundx=0.0, const double boundy=0.0, const double boundz=0.0,
# // bounds on rotation x y z
# const double boundrx=0.0, const double boundry=0.0, const double
# boundrz=0.0){
