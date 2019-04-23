# -*- coding: utf-8 -*-
#author; sgl 
#date: 20190417

#from PyQt5.QtGui import QImage, QPixmap
#from PyQt5.QtCore import QThread, pyqtSignal#, QWaitCondition, QMutex
#from PyQt5.QtWidgets import   QApplication 

import numpy as np
#from PyQt5.QtWidgets import QImage
#import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
from pylsd.lsd import lsd
from camPort import *
"""
In this version, we use 
1,Qthread to control the camera stream
(Qthread is necessary, 
Qtimer is useful, but occur picture slash when your picture processing takes long), 
2,openCV to deal with the frame(easy to add other frame operation algorithm),
3,QLabel to show the stream (pyqtgraph or QVideo is also useful) .

Qthread ref: 
1,https://blog.csdn.net/jeekmary/article/details/88739498
2,https://blog.csdn.net/qq_34710142/article/details/80936986
3,https://www.youtube.com/watch?v=dcSsjxhazu0
Pyserial ref:
1.https://fitsir.me/2016/10/01/A-Python-Serial-Tool/
Cam frame ref:
1,https://blog.csdn.net/weixin_43008870/article/details/86080409
2,https://blog.csdn.net/weixin_43008870/article/details/86496263
UNsolved:   when press close btn, log -viewer error msg:
opencv :
1,https://www.jianshu.com/p/2b79012c0228
QLabel for QPixmap
1,https://blog.csdn.net/oscar_liu/article/details/81210301
2,http://www.pianshen.com/article/3884113106/
QTimer example:
1,http://benhoff.net/face-detection-opencv-pyqt.html   (face detection)
2,https://www.kurokesu.com/main/2016/08/01/opencv-usb-camera-widget-in-pyqt/  (camera calibration)
pyqtgraph & how to  organize Camera class:
1,https://www.pythonforthelab.com/blog/step-by-step-guide-to-building-a-gui/
"""

def eucldist_vectorized(l1,l2):
    l1 = np.array(l1)
    l2 = np.array(l2)
    return np.sqrt(np.sum((l1-l2)**2))

class BaseCamera(Cam):
    def __init__(self, width = 359 ,  height = 305 ):
        super(BaseCamera, self).__init__()
#        self.cap = None
        self.cur_lines = []
        self.pre_lines = []
        self.cur_frame = np.array([])
        self.width = width
        self.height = height
       
    def get_frame(self):
        return self.cur_frame
    
    def frame_processing(self,  frame):
        """
        lsd line extraction
        """
        use_lines = []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        lines = lsd(gray)  ##################  引用pylsd 包的部分r
#        print(lines)
        try:
            for i in range(lines.shape[0]):
                pt1 = (int(lines[i, 0]), int(lines[i, 1]))
                pt2 = (int(lines[i, 2]), int(lines[i, 3]))
                width = lines[i, 4]

                if( (eucldist_vectorized(pt1, pt2) < 50) | (width < 10.0) ):
                    continue 
                use_lines.append(lines[i])
                final_frame = cv2.line(frame, pt1, pt2, (0, 0, 255), int(np.ceil(width / 2)))
            return final_frame,  use_lines   
        except :
            return frame,  lines
    
    def update(self):
        #1, cap new frame
        ret,  frame = self.cap_next_frame()
        #2, get frame processing
        extract_frame, use_lines = self.frame_processing( frame )
        extract_frame = cv2.cvtColor(extract_frame,  cv2.COLOR_BGR2RGB)
        #3, set variables
        self.pre_lines = self.cur_lines
        self.pre_raw_frame = self.cur_raw_frame
        self.cur_raw_frame = frame
        self.cur_lines = use_lines
        self.cur_frame = extract_frame
        
    def __str__(self):
        return 'base camera {}'.format(self.cam_num)

class HeadCamera(Cam):
    def __init__(self, width = 359 ,  height = 305 ):
        super(HeadCamera, self).__init__()
#        self.cap = None
        self.cur_frame = np.array([])
        self.width = width
        self.height = height
       
    def get_frame(self):
        return self.cur_frame
    
    def frame_processing(self,  frame):
        """
        head camera
        """
        print("111")
        final_frame = frame
        features = []
        print("22")

#        return final_frame,  features
        return final_frame,  features
    
    def update(self):
        #1, cap new frame
        ret,  frame = self.cap_next_frame()
        #2, get frame processing
        extract_frame, fea = self.frame_processing( frame )
        try:
            extract_frame = cv2.cvtColor(extract_frame,  cv2.COLOR_BGR2RGB)
        except:
            print("no extract")
        self.cur_frame = extract_frame
        
    def __str__(self):
        return 'base camera {}'.format(self.cam_num)

if __name__=="__main__":
    cam = Camera()
    cam.initialize_cv("/dev/video0")
    print(cam)
    frame = cam.get_frame()
    print(frame)
    cam.close_camera()
