#!/usr/bin/env python
from dynamixel_interface import *
from scipy.spatial.transform import Rotation as R
from time import sleep

def control(J, T, q):
    read_q = np.array([0.5, 0.5, 0.5, 0.5], dtype = np.float64)
    dt = 0.1
    # startComms()
    # read_q = readAngles()
    current_rot, current_xyz = fkine(T, q, read_q) #Obtain current position
    rotation = R.from_dcm(current_rot)
    current_rpy = rotation.as_euler('xyz')
    desired_xyz = np.array(input("Type desired xyz in a comma separated string\n Current position is %5f, %5f, %5f\n" % (current_xyz[0], current_xyz[1], current_xyz[2])))
    # desired_rpy = input("Type desired rpy in a comma separated string")
    desired_rpy = np.array([0,0,0])
    # Weight matrices for the error
    xyz_weight = np.array([5,5,5])
    rpy_weight = np.array([0,0,0])
    loop = True
    apply_correction = True
    while(loop):
        error_xyz_weighted = (desired_xyz - current_xyz)*xyz_weight
        error_xyz = (desired_xyz - current_xyz)
        error_rpy_weighted = (desired_rpy - current_rpy)*rpy_weight
        error_rpy = (desired_rpy - current_rpy)
        error = np.linalg.norm(np.float64(np.hstack((error_xyz, error_rpy_weighted))))
        if error<1e-2:
            desired_xyz = np.array(input("Type desired xyz in a comma separated string\n"))
            # desired_rpy = input("Type desired rpy in a comma separated string")
        else:
            q_dict = dict(zip(q, read_q))
            J_eval = np.array(J.subs(q_dict), dtype=np.float64)
            J_inv = np.linalg.pinv(J_eval)
            error_vect = np.vstack((np.vstack(error_xyz_weighted), np.vstack(error_rpy_weighted)))
            q_dot = J_inv.dot(error_vect)
            q_command = read_q + np.hstack(q_dot)*dt
            # commandAngles(q_command)
            sleep(0.1)
            # read_q = readAngles()
            read_q = q_command
            current_rot, current_xyz = fkine(T, q, read_q)  # Obtain current position
            rotation = R.from_dcm(current_rot)
            current_rpy = rotation.as_euler('xyz')
            print("Current xyz: {}, {}, {}".format(current_xyz[0], current_xyz[1], current_xyz[2]))
            print("Current xyz error: {}, {}, {}".format(error_xyz[0], error_xyz[1], error_xyz[2]))


def fkine(T, q, q_eval): #Forward kinematics
    q_dict = dict(zip(q, q_eval))
    T_eval = np.array(T.subs(q_dict), dtype=np.float64)
    xyz = T_eval[0:3, -1]
    rot = T_eval[0:3, 0:3]
    return (rot, xyz)