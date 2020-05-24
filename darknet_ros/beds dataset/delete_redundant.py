import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import glob
import os
import numpy as np

i=0
inputFolder = 'resized_images'
outputFolder= 'final_images'

for img in glob.glob(inputFolder + '/*.*'):

    try :
        img_d1 = cv2.imread(img)
        j=0
        for img2 in glob.glob(outputFolder + '/*.*'):
            img_d2 = cv2.imread(img2)
            difference = cv2.subtract(img_d1,img_d2)
            b, g, r = cv2.split(difference)
            if cv2.countNonZero(b) == 0 or cv2.countNonZero(g) == 0 or cv2.countNonZero(r) == 0:
                j=1
        
        if (j==0) :
            i+=1
            print(i)
            cv2.imwrite('final_images/image%f.jpg' %(i),img_d1)
            
        cv2.waitKey(1)
    except Exception as e:
            print (e)





