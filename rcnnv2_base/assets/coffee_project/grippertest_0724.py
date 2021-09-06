import cv2
import os
import sys
import numpy as np
from niryo_one_tcp_client import *
from niryo_one_camera import *
import utils

sys.path.append("C:/Users/hoon9/AppData/Local/Programs/Python/Python38")

# Communication Configuration
robot_ip_address = "192.168.0.94"  # Replace by robot ip address

# Initial Pose
init_pose = True
observation_pose = PoseObject(  # position for the robot to watch the workspace
    x=0.20, y=0, z=0.4,
    roll=0.0, pitch=math.pi / 2 + 0.05, yaw=0.0,
)

# Connecting to robot
client = NiryoOneClient()
client.connect(robot_ip_address)

try:
    client.calibrate(CalibrateMode.AUTO)
    client.wait(3)
except:
    print("calibration failed")

# Initial Robot Settings
try:
    client.change_tool(RobotTool.GRIPPER_3)
    client.wait(1)
    print("Gripper Initiated")
except:
    client.wait(1)
    print("Gripper Not Initiated")

try:
    client.open_gripper(RobotTool.GRIPPER_3, 500)
    print("Opening Gripper")
    client.wait(1)
    client.close_gripper(RobotTool.GRIPPER_3, 500)
    print("Closing Gripper")
    client.wait(1)
except:
    print("Gripper Not Working")

client.set_learning_mode(True)
client.quit()