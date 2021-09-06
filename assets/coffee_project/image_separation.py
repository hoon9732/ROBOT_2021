import sys
import os
import shutil
import cv2

src = "C:/Users/hoon9/Documents/SNU/AI_Internship/opencv/videos/capsules"
src2 = "C:/Users/hoon9/Documents/SNU/AI_Internship/opencv/images/capsules_frame_sparse"

for mov in os.listdir(src):
    movpath = os.path.join(src, mov)
    vidcap = cv2.VideoCapture(movpath)
    os.mkdir(os.path.join(src2, mov.replace('.mp4', '')).replace('\\', '/'))
    success, img = vidcap.read()
    cnt = 0
    while success:
        roi = img[62:-62, 0:-1]
        print(os.path.join(src2, mov.replace('.mp4', ''), 'f_'+str(cnt)+'.jpg').replace('\\', '/'))
        if cnt % 10 == 0:
            cv2.imwrite(os.path.join(src2, mov.replace('.mp4', ''), 'f_'+str(cnt)+'.jpg').replace('\\', '/'), roi)
            success, img = vidcap.read()
            print('Reading frame # ', cnt)
        else:
            success, img = vidcap.read()
        cnt += 1