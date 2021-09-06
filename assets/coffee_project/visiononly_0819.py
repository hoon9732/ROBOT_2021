import cv2
import os
import sys
sys.path.append("C:/Users/hoon9/AppData/Local/Programs/Python/Python38")
sys.path.append("C:/Users/hoon9/Documents/SNU/AI_Internship/opencv/assets")

import math
import numpy as np
from niryo_one_tcp_client import *
from niryo_one_camera import *

sys.path.append("C:/Users/hoon9/AppData/Local/Programs/Python/Python38")

# Communication Configuration
robot_ip_address = "192.168.0.33"  # Replace by robot ip address
robot_ethernet_address = "169.254.200.200"
robot_hotspot_address = "10.10.10.10"
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

# Connecting to robot
client=NiryoOneClient()
client.connect(robot_ethernet_address)

# Obtaining image
thres = 0.35 # Threshold to detect object
camwidth = 1280
camheight = 720
brightness = 70

#capture video
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(3, camwidth)
cap.set(4, camheight)
cap.set(10, brightness)

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
    success, img = cap.read()
    classIds, confs, bbox = net.detect(img,confThreshold=thres)
    if len(classIds) != 0:
        for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
            cv2.rectangle(img,box,color=(0,255,0),thickness=2)
            cv2.putText(img,classNames[classId-1].upper(),(box[0]+10,box[1]+30), cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
            cv2.putText(img,str(round(confidence*100,2)),(box[0]+200,box[1]+30), cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
    cv2.imshow("Result", img)
    if cv2.waitKey(25) & 0xFF == ord('q'):
        client.set_learning_mode(True)  
        break