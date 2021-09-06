import cv2
import os
import sys
sys.path.append("C:/Users/hoon9/AppData/Local/Programs/Python/Python38")
sys.path.append("C:/Users/hoon9/Documents/SNU/AI_Internship/opencv/assets")

import math
import numpy as np
from niryo_one_tcp_client import *
from niryo_one_camera import *

# Communication Configuration
#robot_ip_address = "192.168.0.3"  # Replace by robot ip address
robot_ip_address = "169.254.200.200"  # Replace by robot ip address
workspace = "default_workspace"  # Name of your workspace

# Initial Pose

calibration_pose = PoseObject(
    x=0, y=0, z=0.16,
    roll=0, pitch=1.38, yaw=0,
)

calibration_right_pose = PoseObject(
    x=0, y=0, z=0.16,
    roll=0, pitch=1.38, yaw=-1.56,
)

calibration_left_pose = PoseObject(
    x=0, y=0, z=0.16,
    roll=0, pitch=1.38, yaw=1.56,
)

observation_pose = PoseObject(
    x=0.20, y=0, z=0.4,
    roll=0.0, pitch=math.pi / 2 + 0.05, yaw=0.0,
)

drop_pose = PoseObject(
    x=0.20, y=0.20, z=0.10,
    roll=0.0, pitch=math.pi / 2, yaw=0.0,
)

left_quadrant = PoseObject(
    x=0.20, y=-0.20, z=0.4,
    roll=0.0, pitch=math.pi / 2, yaw=0.0,
)

right_quadrant = PoseObject(
    x=0.20, y=0.20, z=0.4,
    roll=0.0, pitch=math.pi / 2, yaw=0.0,
)

holding_pose1 = PoseObject(
    x=0.29, y=-0.09, z=0.36,
    roll=-2.65, pitch=1.55, yaw=-2.53,
)

holding_pose2 = PoseObject(
    x=0.29, y=-0.09, z=0.31,
    roll=-2.65, pitch=1.55, yaw=-2.53,
)


holding_phase3_pose = PoseObject(
    x=0.27, y=0, z=0.41,
    roll=-1.58, pitch=1.52, yaw=-1.67,
)

holding_phase4_pose = PoseObject(
    x=0.27, y=-0.04, z=0.48,
    roll=-1.58, pitch=1.52, yaw=-1.67,
)

picking_phase1_pose = PoseObject(
    x=0.25, y=-0.18, z=0.10,
    roll=1.13, pitch=1.48, yaw=-0.26,
)

picking_phase2_pose = PoseObject(
    x=0.28, y=-0.01, z=0.36,
    roll=-1.31, pitch=1.52, yaw=2.52,
)


# Connecting to robot
client = NiryoOneClient()
client.connect(robot_ip_address)

# Auto Calibration
try:
    client.need_calibration()
    client.calibrate(CalibrateMode.AUTO)
    print("Calibration Complete")
except:
    print("Calibration failed")

try:
    client.change_tool(RobotTool.GRIPPER_3)
    print("Gripper Initiated")
except:
    print("Gripper Not Initiated")

# Initial Pose
client.set_learning_mode(True)
client.wait(3)

# calibrating phase
try:
    client.move_pose(*left_quadrant.to_list())
    print("calibrated")
except:
    print("Calibrating Phase obstructed")

# holding phase
try:
    client.move_pose(*holding_pose1.to_list())
    client.open_gripper(RobotTool.GRIPPER_3, 500)
    client.move_pose(*holding_pose2.to_list())
    client.close_gripper(RobotTool.GRIPPER_3, 500)
    client.move_pose(*holding_pose1.to_list())
    client.set_learning_mode(True)
except:
    print("Holding Phase obstructed")
    client.set_learning_mode(True)
    client.quit()

# # picking phase
# try:
#     # client.open_gripper(RobotTool.GRIPPER_3, 500)
#     # client.move_pose(*left_quadrant.to_list())
#     # client.move_pose(*picking_phase1_pose.to_list())
#     # client.close_gripper(RobotTool.GRIPPER_3, 500)
#     # client.move_pose(*left_quadrant.to_list())
#     # client.move_pose(*picking_phase2_pose.to_list())
#     # client.open_gripper(RobotTool.GRIPPER_3, 500)
#     # client.move_pose(*left_quadrant.to_list())
#     # client.move_pose(*observation_pose.to_list())
#     # client.close_gripper(RobotTool.GRIPPER_3, 500)
#     # client.set_learning_mode(True)
# except:
#     print("Holding Phase obstructed")
#     client.set_learning_mode(True)
#     client.quit() 

