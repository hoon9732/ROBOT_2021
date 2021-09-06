import sys
import os
import shutil
import cv2

src = "C:/yolov5_base/workspace/training_demo/videos/0830"
src2 = "C:/yolov5_base/workspace/training_demo/images/capsules_0830"

for mov in os.listdir(src):
    movpath = os.path.join(src, mov)
    vidcap = cv2.VideoCapture(movpath)
    os.mkdir(os.path.join(src2, mov.replace('.mp4', '')).replace('//', '/'))
    success, img = vidcap.read()
    cnt = 0
    while success:
        roi = img[62:-62, 0:-1]
        print(os.path.join(src2, mov.replace('.mp4', ''), 'f_'+str(cnt)+'.jpg').replace('//', '/'))
        if cnt % 10 == 0:
            cv2.imwrite(os.path.join(src2, mov.replace('.mp4', ''), 'f_'+str(cnt)+'.jpg').replace('//', '/'), roi)
            success, img = vidcap.read()
            print('Reading frame # ', cnt)
        else:
            success, img = vidcap.read()
        cnt += 1