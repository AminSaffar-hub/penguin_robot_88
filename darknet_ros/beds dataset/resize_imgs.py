import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import glob
import os
from PIL import Image

i=0
#inputFolder = '/catkin_ws/src/object detection/yolo_object_detection/dataset/part1'
inputFolder = 'part7'

for img in glob.glob(inputFolder + '/*.*'):

    try :
        ori_img = cv2.imread(img)
        height, width, depth = ori_img.shape
        res_img= cv2.resize(ori_img,(400,400))
        cv2.imshow("Show by CV2",res_img)
        i+=1
        print(i)
        cv2.imwrite('resizepart1/image%f.jpg' %(i),res_img)
        cv2.waitKey(1)
    except Exception as e:
            print (e)
