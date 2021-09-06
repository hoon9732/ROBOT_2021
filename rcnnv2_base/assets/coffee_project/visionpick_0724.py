import cv2
import os
import sys
import math
import numpy as np
from niryo_one_tcp_client import *
from niryo_one_camera import *
import utils

sys.path.append("C:/Users/hoon9/AppData/Local/Programs/Python/Python38")

# Communication Configuration
robot_ip_address = "192.168.0.94"  # Replace by robot ip address
workspace = "default_workspace"  # Name of your workspace

# Initial Pose
observation_pose = PoseObject(  # position for the robot to watch the workspace
    x=0.20, y=0, z=0.4,
    roll=0.0, pitch=math.pi / 2 + 0.05, yaw=0.0,
)

drop_pose = PoseObject(  # position for the robot to watch the workspace
    x=0.20, y=0.20, z=0.10,
    roll=0.0, pitch=math.pi / 2, yaw=0.0,
)

left_quadrant = PoseObject(  # position for the robot to watch the workspace
    x=0.20, y=-0.20, z=0.4,
    roll=0.0, pitch=math.pi / 2 + 0.05, yaw=0.0,
)

right_quadrant = PoseObject(  # position for the robot to watch the workspace
    x=0.20, y=0.20, z=0.4,
    roll=0.0, pitch=math.pi / 2 + 0.05, yaw=0.0,
)

test1_pose = PoseObject(  # position for the robot to watch the workspace
    x=0.11, y=0, z=0.37,
    roll=0.0, pitch=-1.40, yaw=0.0,
)

test2_pose = PoseObject(  # position for the robot to watch the workspace
    x=0.14, y=0, z=0.09,
    roll=0.16, pitch=0.1, yaw=0.17,
)

# Connecting to robot
client = NiryoOneClient()
client.connect(robot_ip_address)

font = cv2.FONT_HERSHEY_SIMPLEX

# Initiate Learning Mode
try:
    client.set_learning_mode(True)
    client.wait(1)
except:
    print("learning mode failed")

# Auto Calibration
try:
    client.calibrate(CalibrateMode.AUTO)
except:
    print("calibration failed")

try:
    client.change_tool(RobotTool.GRIPPER_3)
    print("Gripper Initiated")
except:
    print("Gripepr Not Initiated")

try:
    client.open_gripper(RobotTool.GRIPPER_3, 500)
    print("Opening Gripper")
    client.close_gripper(RobotTool.GRIPPER_3, 500)
    print("Closing Gripper")
except:
    print("Gripper Not Working")

# Joint Moving Test
# try:
#     client.move_joints(1, 0, 0 , 0, 0, 0)
#     client.move_joints(-1, 0, 0 , 0, 0, 0)
# except:
#     print("Joint Moving Failed")

# Obtaining image
thres = 0.35 # Threshold to detect object
camwidth = 1280
camheight = 720
brightness = 70

# Model Loading
classNames = []
cupclasses = [34, 47, 51]
classFile = "coco.names" # namelist of the COCO API https://cocodataset.org/#download
with open(classFile,"rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

for i in range(len(classNames)):
    if classNames[i] == 'frisbee':
        classNames[i] = 'cup'
    elif classNames[i] == 'bowl':
        classNames[i] = 'cup'
    else:
        pass

configPath = "ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)



#move to observation_pose
init_pose = True
while init_pose:
    client.move_pose(*observation_pose.to_list())
    init_pose = False

while True:
    #capture video
    imagecheck, img_comp = client.get_img_compressed()
    img_raw = uncompress_image(img_comp)
    img = img_raw
    img = cv2.resize(img_raw, dsize=(camwidth, camheight), interpolation=cv2.INTER_AREA)
    classIds, confs, bbox = net.detect(img,confThreshold=thres)
    print(classIds)
    # try:
    #     if classIds[0][0] not in cupclasses:
    #         client.move_pose(*left_quadrant.to_list())
    #     elif classIds[0][0] not in cupclasses and j1cnt >= 1:
            
    #         pass
    # except:
    #     client.shift_pose(RobotAxis.Y, 0.05)
    #print(classIds,bbox)
    # if classIds[0] in cupclasses:
    #     print("Initial")
    #     print(bbox)
    #     if len(boxmean) < 1:
    #         boxmean = bbox
    #         print("Start")
    #     else:
    #         for i in range(len(bbox)):
    #             print(i, "th element is", boxmean[i], bbox[i])
    #             boxmean[i] = list(str(boxmean[i]).replace(" ", " ,"))
    #             bbox[i] = list(str(bbox[i]).replace(" ", " ,"))
    #             print(i, "th element is", boxmean[i], bbox[i])
    #             arr[i] = np.mean(np.array(boxmean[i]),np.array(bbox[i]))
    #             boxmean[i] = arr[i].tolist()
    #         print("Calculation Complete")
    #         print(boxmean)
    if len(classIds) != 0:
        for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
            cv2.rectangle(img,box,color=(0,255,0),thickness=2)
            cv2.putText(img,classNames[classId-1].upper(),(box[0]+10,box[1]+30), cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
            cv2.putText(img,str(round(confidence*100,2)),(box[0]+200,box[1]+30), cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
    cv2.imshow("Result", img)
    if cv2.waitKey(25) & 0xFF == ord('q'):
        # client.set_learning_mode(True)
        client.quit()   
        break