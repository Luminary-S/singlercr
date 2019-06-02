#!./venv python
# -*- coding: utf-8 -*-
#import numpy as np
from serialPort import *
import re
#import serial
from SerialThread import *
from ur_move import  *

"""
two clases are defined in the file.
URmove gives the ros  

URThread
"""

class URmove():
    def __init__(self):   
        # super(Sensor, self).__init__()
        self.ur_status = "stop"
        self.switch = {
            "demo": self.path_demo_move,
            # "valueB": functionB,
            # "valueC": functionC
        }

    def set_status(self, status):
        self.ur_status = status

    def path_demo_move(self, str):
        print("path demo move "+ str)
        pass

    def update(self):
        #1, read com data
        # data = self.read_data()
        status = self.ur_status
        try:
            self.switch[status]("ok")
        except KeyError as e:
            pass

        #2, decode data and set values

    
#    def update(self):    
    def __str__(self):
        print("ur move ")

class URscript():
    def __init__(self, pub):
        self.pub = pub

    def pub(self,str):
        self.pub.publish(str)

    def movej(self, qq, vel, ace, t):
        ss = "movej([" + str(qq[0]) + "," + str(qq[1]) + "," + str(qq[2]) + "," + str(qq[3]) + "," + str(
            qq[4]) + "," + str(qq[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(vel) + "," + "t=" + str(t) + ")"
        print("---------ss:", ss)
        return ss

    def movel(self, qq, vel, ace, t):
        ss = "movel([" + str(qq[0]) + "," + str(qq[1]) + "," + str(qq[2]) + "," + str(qq[3]) + "," + str(
            qq[4]) + "," + str(qq[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(vel) + "," + "t=" + str(t) + ")"
        print("---------ss:", ss)
        return ss

    def stopj(self, pub, a):
        ss = "stopj(" + a + ")"
        print("---------ss:", ss)
        return ss

'''ur thread '''
class URThread(SerialThread):

    urSignal = pyqtSignal()

    def __init__(self):
        super(URThread, self).__init__()
        self.device = URmove()

    def run(self):
        while 1:
            self.mutex.lock()
            # print("isstop:",self.isstop)
            if self.isstop:
                self.cond.wait(self.mutex)
            self.device.update()
            self.controlSignal.emit()
            self.mutex.unlock()
#
def test():
    uThread = URThread()
    uThread.start()
    uThread.resume()

if __name__=="__main__":
    test()
    
