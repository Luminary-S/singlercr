#!/usr/bin/python3
# -*- coding: utf-8 -*-
#author;sgl 
#date: 20190417

import numpy as np
import cv2

from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QThread, pyqtSignal#, QWaitCondition, QMutex

class Cam():
    def __init__(self):
        self.cam_num = 0  # default open video0
        self.cap = None
        self.cur_raw_frame = np.array([])
        self.pre_raw_frame = np.array([])
    
    def set_port(self,  port):
        self.cam_num = port
        
    def get_frame(self):
        return self.cur_raw_frame
    
    def initialize_cv(self,  port):
        self.set_port(port)
        self.cap = cv2.VideoCapture( self.cam_num )
    
    def initialize_ros(self):
        pass
    
    def cap_next_frame(self):
        """
        capture frame and convert it into a RGB frame, so thant other tools can be used
        """
        try:
#            print("111")
            ret,  frame = self.cap.read()
#            print("222")
            return ret,  frame
#        if( ret == True):
##            cur_frame = cv2.cvtColor(readFrame,  cv2.COLOR_BGR2RGB)
#            return ret,  frame     
        except:
            print("no frame!")  
    
    def frame_processing(self,  frame):
        """ can be rewriten by different cam processing mthod"""
        pass    
   
    def update(self):
        pass
 
    def close_camera(self):
#        print("in")
        self.cap.release()
        cv2.destroyAllWindows()
#        print("out")
        
    def __str__(self):
        return 'open camera {}'.format(self.cam_num)
        
'''cam thread '''
class CamThread(QThread):
    #define a signal, when cam opens, emit this signal
    camSignal = pyqtSignal(int)
    def __init__(self,  cam, label):
        super(CamThread,  self).__init__()
#        self.tnum = num
        self.isstop = True
        self.cam = cam
        self.label = label
#        self.cond = QWaitCondition()
#        self.mutex = QMutex()
        self.img = np.array([])
        self.i=0
#        self.camSignal.connect( change_label_pic )
            
    def __del__(self):
        #线程状态改变与线程终止
        self.isstop = True
        self.quit()

    def pause(self):
        self.isstop = False
        self.quit()
        
    def resume(self):
        self.isstop = True
#        self.cond.wakeAll()
    
    def convert_to_qimage(self,  frame):
        h,  w = frame.shape[:2]
        img = QImage(frame, 
                                        w, h,  QImage.Format_RGB888)
        img = QPixmap.fromImage(img)
        return img
    
    def change_label_pic(self):
        try:
            frame = self.cam.get_frame()
            self.img = self.convert_to_qimage(frame)
            self.label.setPixmap(self.img)
            self.label.setScaledContents(True)
        except:
            print("no img"+str(self.i))
            return  None          
     
    def run(self):
        while self.isstop:
            self.cam.update()
            self.i+=1
            self.camSignal.emit(self.i)
            
#def convert_to_qimage(frame):
#    h,  w = frame.shape[:2]
#    img = QImage(frame, 
#                                    w, h,  QImage.Format_RGB888)
#    img = QPixmap.fromImage(img)
#    return img
#
#def change_label_pic( label,  frame ):
#    try:
#        img = convert_to_qimage(frame)
#        label.setPixmap(img)
#        label.setScaledContents(True)
#    except:
#        print("no img")
#        return
from pylsd.lsd import lsd

def eucldist_vectorized(l1,l2):
    l1 = np.array(l1)
    l2 = np.array(l2)
    return np.sqrt(np.sum((l1-l2)**2))

def main():
    cap = cv2.VideoCapture(0)  ##############  如果不是正确的camera的图像，改动0为1,2,3,这样去试就行
    while True:

        ret, frame = cap.read()
        final  = frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        lines = lsd(gray)  ##################  引用pylsd 包的部分
        for i in range(lines.shape[0]):
            pt1 = (int(lines[i, 0]), int(lines[i, 1]))
            pt2 = (int(lines[i, 2]), int(lines[i, 3]))
            width = lines[i, 4]

            if( (eucldist_vectorized(pt1, pt2) < 50) | (width < 10.0) ):
                continue
            final=cv2.line(frame, pt1, pt2, (0, 0, 255), int(np.ceil(width / 2)))

        cv2.imshow("after few lines", final)

        ###按下q，就可以关闭窗口
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # cv2.waitKey(0)

    cap.release()
    cv2.destroyAllWindows()
    
if __name__=="__main__":
    main()
