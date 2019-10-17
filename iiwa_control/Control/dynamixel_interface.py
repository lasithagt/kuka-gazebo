#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Read and Write Example      *********
#
#
# Available Dynamixel model on this example : All models using Protocol 2.0
# This example is designed for using a Dynamixel PRO 54-200, and an USB2DYNAMIXEL.
# To use another Dynamixel model, such as X series, see their details in E-Manual(emanual.robotis.com) and edit below variables yourself.
# Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
#

# import os
#
# if os.name == 'nt':
#     import msvcrt
#     def getch():
#         return msvcrt.getch().decode()
# else:
# import sys, tty, termios
# fd = sys.stdin.fileno()
# old_settings = termios.tcgetattr(fd)
#
#
# def getch():
#     try:
#         tty.setraw(sys.stdin.fileno())
#         ch = sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#         return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import numpy as np

# Control table address
ADDR_ID = 7
ADDR_PRO_TORQUE_ENABLE = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION = 116              # CW is decreasing, CCW is increasing
ADDR_PRO_PRESENT_POSITION = 132
ADDR_MIN_POSITION_LIMIT = 52
ADDR_MAX_POSITION_LIMIT = 48
ADDR_OPERATING_MODE = 11
ADDR_DRIVE_MODE = 10
ADDR_HOMING_OFFSET = 20
ADDR_POS_P_GAIN = 84
ADDR_POS_D_GAIN = 80
ADDR_POS_I_GAIN = 82
ADDR_PROFILE_ACC = 108
ADDR_PROFILE_VEL = 112





# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_IDs                      = np.array([4])#np.arange(1,6)                 # Dynamixel IDs
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 10           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold



# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# while 1:
#     print("Press any key to continue! (or press ESC to quit!)")
#     if getch() == chr(0x1b):
#         break
#
#     # Write goal position
#     dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index])
#     if dxl_comm_result != COMM_SUCCESS:
#         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#     elif dxl_error != 0:
#         print("%s" % packetHandler.getRxPacketError(dxl_error))

    # while 1:
    #     # Read present position
    #     dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))
    #
    #     print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position[index], dxl_present_position))
    #
    #     if not abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
    #         break


def startComms():
    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press enter to terminate...")
        raw_input()
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press enter to terminate...")
        raw_input()
        quit()

def setHomingOffset(id, offset):
    rad_per_value = 0.088 * (np.pi / 180)
    offset = offset/rad_per_value
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id,
                                                              ADDR_HOMING_OFFSET, offset)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

# Enable Dynamixel Torque
def enableTorque():
    for id in DXL_IDs:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel %03d has been successfully connected" % id)

def disableTorque():
    for id in DXL_IDs:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE,
                                                                  TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

def readAngles():
    rad_per_value = 0.088*(np.pi/180)
    q_read = np.zeros([len(DXL_IDs)])
    count = 0
    for id in DXL_IDs:
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, id,
                                                                                       ADDR_PRO_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        q_read[count] = dxl_present_position
        count = count + 1
    return q_read*rad_per_value

def commandAngles(IDs, q_command):

    if q_command is not np.ndarray:
        q_command = np.array([q_command])

    rad_per_value = 0.088*(np.pi/180)
    q_command = np.array(q_command/rad_per_value, dtype=np.int)
    if len(IDs) == len(q_command):
        count = 0
        for id in IDs:
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_PRO_GOAL_POSITION,
                                                                      q_command[count])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
                count = count + 1
    else:
        print("Number of IDs and commanded angles doesn't match")

def setAngleBounds(id, min_position, max_position):
    rad_per_value = 0.088 * (np.pi/180)
    min_position = int(np.rint(min_position/rad_per_value))
    max_position = int(np.rint(max_position/rad_per_value))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id,
                                                              ADDR_MIN_POSITION_LIMIT,min_position)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id,
                                                              ADDR_MAX_POSITION_LIMIT, max_position)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

def setProfileVelocity(id, velocity):
    #Convert from radians per second to units
    conv_fact = 1/0.229
    velocity = velocity*conv_fact
    velocity = int(velocity)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id,
                                                              ADDR_PROFILE_VEL, velocity)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

def setProfileAcceleration(id, acceleration):
    #Convert from radians per second^2 to units
    conv_fact = 1/214.577
    acceleration = int(acceleration*conv_fact)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id,
                                                              ADDR_PROFILE_ACC, acceleration)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

def endComms():
    portHandler.closePort()
