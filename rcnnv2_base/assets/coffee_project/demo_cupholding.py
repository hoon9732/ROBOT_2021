#!/usr/bin/env python

# To use the API, copy these 4 lines on each Python file you create
from niryo_one_python_api.niryo_one_api import *
import rospy
import time
import math

rospy.init_node('niryo_one_example_python_api')

print "--- Start"

n = NiryoOne()

try:
    # Calibrate robot first
    n.calibrate_auto()
    #n.calibrate_manual()
    print "Calibration finished !\n"

    time.sleep(1)

    # Test learning mode(Torque_Off)
    n.activate_learning_mode(True)
    
    # Coordinates
    c=[]
    c.append([0.11, 0, 0.37, 0, -1.40, 0])
    c.append([0.14, 0, 0.09, 0.16, 0.1, 0.17])
        
    # Move_Phase_1(Raising)
    n.set_arm_max_velocity(30)
    n.move_pose(c[0][0], c[0][1], c[0][2], c[0][3], c[0][4], c[0][5])

    # Test gripper
    n.change_tool(TOOL_GRIPPER_2_ID)
    print "\nCurrent tool id:"
    print n.get_current_tool_id()

    n.wait(5)
    n.open_gripper(TOOL_GRIPPER_2_ID,500)

    # Move_Phase_2(Picking)
    n.move_pose(c[1][0], c[1][1], c[1][2], c[1][3], c[1][4], c[1][5])
    n.close_gripper(TOOL_GRIPPER_2_ID,500)

    n.close_gripper(TOOL_GRIPPER_2_ID,500)

    #n.change_tool(TOOL_NONE) # Unmount

    # Test learning mode(Torque_Off)
    n.activate_learning_mode(True)

except NiryoOneException as e:
    print e
    # handle exception here
    # you can also make a try/except for each command separately

print "--- End"