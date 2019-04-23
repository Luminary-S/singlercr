# -*- coding: utf-8 -*-

import serial
import serial.tools.list_ports
import re

''' base class of serial port,  
it is the base class for sensor board and motor controller Arduino board
'''
class SerialPort(object):
    def __init__(self):
        self.ComDict = {}
        self.ser = serial.Serial()
    
    def set_port(self,  port,  rate):
        self.ser.port = port
        self.ser.baudrate = rate
        self.ser.byteSize = serial.EIGHTBITS
        self.ser.stopbits=serial.STOPBITS_ONE
        self.ser.parity= serial.PARITY_NONE
        self.ser.timeout = 0.5
    
    def get_port(self):
        if self.ser.isOpen():
            return self.ser.port
        else:
            print("not open port")
            return None
        
    def detect_port(self):
        self.ComDict={}
        # 1, get all port 
        port_list  = list(serial.tools.list_ports.comports()) 
        #2, clear all port box
#        self.video_com_box.clear()
        #3, show the insert port com
        for port in port_list:
            self.ComDict["%s" % port[0]] = "%s" % port[1]
#            self.video_com_box.addItem(port[0])
        #4, error handle, msgbox show
        if len(self.ComDict) == 0:
            print("no com port!")
    
    def init_port(self,  port,  rate):
        self.set_port(  port,  rate )
        self.open_port()
        self.flush()
                
    def open_port(self):
#        self.set_port(port)
        try:
            self.ser.open()
        except:
            print(" open error!")
#            return None
    
    def close_port(self):
        self.ser.close()
    
    def flush(self):
        self.ser.readline()
    
    def read_data(self):
        try:
            data = self.ser.readline()
            print("ser read: ",  data)
            data = data.decode()
        except:
            data=""
        return data
    
    def send_data(self,  data):
        if self.ser.isOpen():
            self.ser.write( data )
            print("send data:"+ str(data))
            
def test_serialport():
    ser = SerialPort()
    #1, detect port
    ser.detect_port()
    #2, set port
    ser.set_port(  list( ser.ComDict.keys()) [0] )

    #3, open port
    ser.open_port()
    print(ser.get_port())
    #4. read data
    ser.flush()
    data = ser.read_data()
    #5. data process
#    data = data.split("sz|su")
    print(data)
    data = re.split("sz|su|\n",  data )
    print("data:")
    print(data[1:-1])
    
#    pass

if __name__=="__main__":
    test_serialport()

