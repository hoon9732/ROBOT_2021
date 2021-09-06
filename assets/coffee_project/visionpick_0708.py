
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
workspace = "default_workspace"  # Name of your workspace

# Initial Pose
init_pose = True
observation_pose = PoseObject(  # position for the robot to watch the workspace
    x=0.20, y=0, z=0.4,
    roll=0.0, pitch=math.pi / 2 + 0.05, yaw=0.0,
)

drop_pose = PoseObject(  # position for the robot to watch the workspace
    x=0.20, y=0.20, z=0.10,
    roll=0.0, pitch=math.pi / 2, yaw=0.0,
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

try:
    client.calibrate(CalibrateMode.AUTO)
    client.wait(3)
except:
    print("calibration failed")

client.move_pose(*test1_pose.to_list())
print("Moving to Pose1")
client.move_pose(*test2_pose.to_list())
print("Moving to Pose2")

# Initial Robot Settings
client.change_tool(RobotTool.GRIPPER_2)
print("Gripper Initiated")
client.wait(3)
client.open_gripper(RobotTool.GRIPPER_2,2000)
print("Opening Gripper")
client.wait(3)
client.close_gripper(RobotTool.GRIPPER_2,2000)
print("Closing Gripper")
client.wait(3)

# Obtaining image
thres = 0.50 # Threshold to detect object
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

while init_pose:
    #move to observation_pose
    client.move_pose(*observation_pose.to_list())
    init_pose = False

boxmean = []
arr = []
while True:
    #capture video
    imagecheck, cap = utils.take_workspace_img(client)
    img = cv2.resize(cap, (camwidth, camheight))
    classIds, confs, bbox = net.detect(img,confThreshold=thres)
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
        breakQQ



# picking


#     a, obj = client.get_target_pose_from_rel(workspace, -0.01, objs[x].x / shape[0], objs[x].y / shape[1],
#                                                 objs[x].angle)
#     client.pick_from_pose(*obj.to_list())
#     client.place_from_pose(*drop_pose.to_list())
#     break



#     #calculate the mask for img_work (black and white image)
#     mask = utils.objs_mask(img_work)
#     #aply the mask to the image
#     img_masked = cv2.bitwise_and(img_work, img_work, mask=mask)

#     img_db = concat_imgs([img_work, mask, img_masked], 1)

#     #get all opbject from the image
#     objs = utils.extract_objs(img_work, mask)
#     if len(objs) == 0:
#         continue

#     imgs = []
#     #resize all objects img to 64*64 pixels
#     for x in range(len(objs)):
#         imgs.append(resize_img(objs[x].img, width=64, height=64))

#     imgs = np.array(imgs)

#     #predict all the images
#     predictions = model.predict(imgs)

#     #for all predictions find the corresponding name and print it on img_db
#     for x in range(len(predictions)):
#         obj = objs[x]
#         pred = predictions[x].argmax()
#         cv2.drawContours(img_db, [obj.box], 0, (0, 0, 255), 2)
#         pos = [obj.square[0][1], obj.square[1][1]]
#         img_db = cv2.putText(img_db, objects_names[pred], tuple(pos), font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
#         pos[0] += img_db.shape[0]
#         img_db = cv2.putText(img_db, objects_names[pred], tuple(pos), font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

#     img_db = resize_img(img_db, width=1200, height=400)

#     show_img("robot view", img_db, wait_ms=50)

#     shape = img_work.shape

#     print(objects_names)
#     #ask to the user the name of the object he want
#     string = input("what do you want? (", +str(objects_names)+")")

#     #find an object with the same name in the model predictions
#     for x in range(len(predictions)):
#         pred = predictions[x].argmax()

#         if objects_names[pred] == string:
#             print("object find")
#             #calculate the position of the object and grab it
#             a, obj = client.get_target_pose_from_rel(workspace, -0.01, objs[x].x / shape[0], objs[x].y / shape[1],
#                                                      objs[x].angle)
#             client.pick_from_pose(*obj.to_list())
#             client.place_from_pose(*drop_pose.to_list())
#             break
