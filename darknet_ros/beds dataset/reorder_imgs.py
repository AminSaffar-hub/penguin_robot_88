import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import glob
import os
from PIL import Image

i=0
#inputFolder = '/catkin_ws/src/object detection/yolo_object_detection/dataset/part1'
inputFolder = 'final_images'

for img in glob.glob(inputFolder + '/*.*'):

    try :
        ori_img = cv2.imread(img)
        cv2.imshow("Show by CV2",ori_img)
        i+=1
        print(i)
        cv2.imwrite('final_dataset/image%f.jpg' %(i),ori_img)
        cv2.waitKey(1)
    except Exception as e:
            print (e)