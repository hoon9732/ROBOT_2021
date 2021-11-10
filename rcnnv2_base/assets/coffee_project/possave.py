import os
import sys
sys.path.append("C:/Users/hoon9/AppData/Local/Programs/Python/Python38")
sys.path.append("C:/Users/hoon9/Documents/SNU/AI_Internship/opencv/rcnnv2_base/assets")

import math
import numpy as np
from niryo_one_tcp_client import *
from niryo_one_camera import *
import utils

# Communication Configuration
#robot_ip_address = "192.168.0.3"  # Replace by robot ip address
robot_ip_address = "192.168.0.3"  # Replace by robot ip address
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
    roll=0.0, pitch=math.pi / 2 + 0.05, yaw=0.0,
)

right_quadrant = PoseObject(
    x=0.20, y=0.20, z=0.4,
    roll=0.0, pitch=math.pi / 2 + 0.05, yaw=0.0,
)

test1_pose = PoseObject(
    x=0.11, y=0, z=0.37,
    roll=0.0, pitch=-1.40, yaw=0.0,
)

test2_pose = PoseObject(
    x=0.14, y=0, z=0.09,
    roll=0.16, pitch=0.1, yaw=0.17,
)

# Connecting to robot
client = NiryoOneClient()
client.connect(robot_ip_address)

# Initiate Learning Mode
try:
    client.set_learning_mode(True)
    client.wait(1)
except:
    print("learning mode failed")

# Auto Calibration
try:
    client.need_calibration()
    client.calibrate(CalibrateMode.AUTO)
except:
    print("calibration failed")

try:
    client.change_tool(RobotTool.GRIPPER_3)
    print("Gripper Initiated")
except:
    print("Gripper Not Initiated")

init_pose = True

client.set_learning_mode(True)
print("Getting Pose")
pose_group = []

while True:
    stat, dat = client.get_pose()
    pose_list = client.pose_to_list(dat)
    if len(pose_group) < 5:
        pose_group.append(pose_list)
    elif np.abs(np.mean(np.array(pose_list) - np.mean(np.array(pose_group), axis = 0))) < 0.001:
        pose_round = [round(x, 2) for x in pose_list]
        print("Final Answer is")
        print(pose_round)
        break
    else:
        del pose_group[0]
        pose_group.append(pose_list)
    client.wait(1)
    print(pose_list)
    if cv2.waitKey(25) & 0xFF == ord('q'):
        # client.set_learning_mode(True)
        client.quit()   
        break