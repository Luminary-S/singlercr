#!/usr/bin/venv python
# -*- coding: utf-8 -*-
#from PyQt5.QtGui import QImage, QPixmap
# from PyQt5.QtCore import QThread, pyqtSignal, QWaitCondition, QMutex
##from PyQt5.QtWidgets import   QApplication 
#
#import numpy as np
from serialPort import *
import re
#import serial
from SerialThread import *

class Sensor(SerialPort):
    def __init__(self):   
        super(Sensor, self).__init__()
        self.force0 = 0
        self.sonic0 = 999
        self.force1 = self.force0
        self.sonic1 = self.force0
    
    def close_sensor(self):
        #1, close port
        self.close_port()
        #2, data clear
        self.force0 = 0
        self.sonic0 = 0
    
    def initialize_sensor(self,  port,  rate ):
        self.init_port( port, rate )

    def get_force(self):
        return self.force1
    
    def get_sonic(self):
        return self.sonic1

    # data decode 
    def decode_data(self,  data):
        """
        force: t100
        sonic:u100
        """
        re_force = r'(t)([0-9]+)'
        re_sonic = r'(u)([0-9]+)'

        # print("original:",  data)
#        re_force = re.compile()
#        re_sonic = re.compile()
#        m_force = self.match_data(data,  re_force)
#        m_sonic = self.match_data(data,  re_sonic)
#        
#        if m_force:
        ### match data based on the re, return the second group(data group)
        m_force = self.match_data( data,  re_force,  2 )
        if m_force:
            self.force1 = m_force
            self.sonic1 = self.sonic0
        else:
            m_sonic = self.match_data( data,  re_sonic,  2 )
            if m_sonic:
                self.sonic1 = m_sonic
                self.force1 = self.force0
#        data = re.split("sz|su|\n",  data )
#        if len(data) >= 4:
#            data = data[1:-1]
#            print("trim:", data)
#            self.force1 = data[0]
#            self.sonic1 = data[1]
        
    
    def update(self):
        #1, read com data
        data = self.read_data()
        #2, decode data and set values
        self.decode_data(data)
        # print("force",self.force1)
        self.force0 = self.force1
        self.sonic0 = self.sonic1
    
#    def update(self):    
    def __str__(self):
        print(" force & ultra sonic sensors")

        
'''sensor thread  for sensor data show'''
class SensorThread(SerialThread):
    sensorSignal = pyqtSignal()

    def __init__(self):
        super(SensorThread, self).__init__()
        self.sensor = Sensor()

    def run(self):
        while 1:
            self.mutex.lock()
            # print("isstop:",self.isstop)
            if self.isstop:
                self.cond.wait(self.mutex)
            self.sensor.update()
            self.sensorSignal.emit()
            self.mutex.unlock()
    #define a signal, when cam opens, emit this signal

#     def __init__(self):
#         super(SensorThread,  self).__init__()
# #        self.tnum = num
#         self.isstop = True
#         self.cond = QWaitCondition()
#         self.mutex = QMutex()
#         self.sensor = Sensor()
#
#     def __del__(self):
#         #线程状态改变与线程终止
#         self.isstop = True
#         self.quit()
#
#     def pause(self):
# #        print("thread pause")
#         self.isstop = True
#
#     def resume(self):
#         self.isstop = False
#         self.cond.wakeAll()
#
def test():
    sensor = Sensor()
    # 1, detect port
    sensor.detect_port()
    # 2, set port
    port = list(sensor.ComDict.keys())[0]
    print(port)
    # 3, open port
    sensor.initialize_sensor(port)
    # 4. read data
    sensor.update()
    # 5. data process
    #    data = data.split("sz|su")
    print("force: ", sensor.get_force())
    print("sonic: ", sensor.get_sonic())

def main():
    # while True:
    sensorT = SensorThread()
    sensorT.start()
    sensorT.sensor.initialize_sensor("/dev/ttyUSB0", 9600)
    sensorT.resume()

    while True:
        # sensorT.start()
        print(sensorT.sensor.sonic1)
    # sensorT.resume()

    
if __name__=="__main__":
    main()
    
