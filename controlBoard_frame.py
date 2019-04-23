#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Copyright (c) 2019.04, Guangli SUN
# All rights reserved.

## cmd file : serial_cmd.txt
from serialPort import *
from rcr import *
#import rospy,time
import serial
#import string
#import math
import yaml#,os,sys


def cmd_cfg_into_dict(fname = 'serial_cmd.txt'):
    dict = {}
    with open(fname,'r') as f:
        for line in f.readlines():
            # print type(line)
            l = line.split(":")
            dict[l[0]] = l[1].split()[0]
    print(dict)
    return dict

def read_cmd_from_yaml( fname = 'cmd.yaml' ):
    rcrmove_dict = {}
    clean_dict = {}
    ur_dict = {}
    f = open(fname)
    yamldata = yaml.load(f)
    with open(fname, 'r') as f:
        yamldata = yaml.load(f)
        for i in yamldata["RCRmove"].split():
            l = i.split(":")
            rcrmove_dict[l[0]] = l[1].split()[0]
        for i in yamldata["clean"].split():
            l = i.split(":")
            clean_dict[l[0]] = l[1].split()[0]
        for i in yamldata["UR"].split():
            l = i.split(":")
            ur_dict[l[0]] = l[1].split()[0]
        # for line in f.readlines():
        #     # print line
        #     l = line.split(":")
        #     dict[l[0]] = l[1].split()[0]
    # print dict
    return rcrmove_dict, clean_dict, ur_dict


class ControlBoard(SerialPort):

    def __init__(self):
        super(ControlBoard, self).__init__()
        self.rcr = RCR(1)
        self.cmd_dict = cmd_cfg_into_dict()
        self.basecam_pitch = 50
        self.basecam_yaw = 100 

    def init(self,  port,  rate ):
        self.set_port(  port,  rate )
        self.open_port()
    """
    operation for each movement cmd, serial com data to arduino ;
    step move
    """
    def cmd_send(self,  cmmd):
        self.ser.write(cmmd.encode())
        
    def cmd_move(self, status):
        print("step:"+status)
        print("cmd:" + self.cmd_dict[status])
        if status =='up':
            h,  v = self.rcr.fixed_update(2000, 200000)
        elif status == "down":
            h,  v = self.rcr.fixed_update(2000, -200000)
        cmd = self.cmd_dict[status]
        self.cmd_send( cmd )
        # time.sleep(0.01)
        # rospy.loginfo("RCR will move up.")
        # pass    
    """
    set move
    """
    def cmd_precise_move(self,  height,  vel ):
        """
        move relative height at the given vel
        sample: number(3000 vel is OK),e,number(100000 pos is OK),f,o,(g,h),d  
        """
#        print("given target: " + str(height) + ", "+  "vel: " + str(vel) )
        h,  v = self.rcr.update(height, vel)
        if height > 0:
            cmd =  str(v)+"e"+str(h) + "fogd" 
            self.cmd_send(cmd)
        else:
            cmd =  str(v)+"e"+str( h ) + "fohd"
            self.cmd_send(cmd)
    
    """
        operation for each movement cmd, serial com data to arduino ;
        """
    def cmd_clean(self, status):
        print("step:"+status)
        print("cmd:" + self.cmd_dict[status])
        cmd = self.cmd_dict[status].encode() 
        self.cmd_send(cmd)
        # time.sleep(0.01)
        # rospy.loginfo("RCR will move up.")
        # pass
    
    """
    camera:
    (left right yaw)steering Vertical axis:(50~210),f,p,g,d
    (pitch)steering Horizontal axis:(100~200),f,q,g,d
    """
    def ser_cam_adjust(self,  pitch,  yaw):
        cmd_pitch = str(pitch)+"fqgd"
        cmd_yaw = str(yaw) + "fpgd"
        
        self.ser.write( cmd_pitch.encode() )
        self.ser.write( cmd_yaw.encode() )
        
    def cmd_cam_adjust(self,  pitch,  yaw):
        pitch = pitch + 100
        yaw = yaw + 50
        self.ser_cam_adjust(pitch, yaw)
        return pitch, yaw
        
    def get_basecam_angles(self):
        return self.basecam_pitch, self.basecam_yaw
    
    def set_init_basecam_angle(self,  pitch,  yaw):
        self.basecam_pitch = pitch + 100
        self.basecam_yaw = yaw + 50
        pitch,  yaw = self.get_basecam_angles()
        return pitch, yaw
    
    def go_init_basecam_angle(self):
        self.ser_cam_adjust( self.basecam_pitch, self.basecam_yaw)
        pitch = self.basecam_pitch -100
        yaw = self.basecam_yaw -50
        return pitch, yaw
        
    def map_value(self, val1,  val2):
        pass

    def ERROR_detection(self, error_info):
        if error_info == "no power":
            self.car_status = "stop"
    
  

def main():
    #1, login in the control board
        port = "/dev/ttyACM0"
        self.ser = serial.Serial(port = port, baudrate = 9600, timeout = 0.5, parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

def test():
    cmd_cfg_into_dict()


if __name__=="__main__":
    test()

