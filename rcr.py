#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Copyright (c) 2019.04, Guangli SUN
# All rights reserved.

## cmd file : serial_cmd.txt
#from serialPort import *
#import rospy,time
#import serial
#import string
#import math
#import yaml#,os,sys

class RCR():
    def __init__(self,  num):
        self.carnum = num
        # for calculation,
        # initial height; pre height; height; desiganate height; calculate height
        self.hd = 0
        self.vd = 0
        self.h0 = 0
        self.v0 = 0
        self.h = 0
        self.v = 0
        self.dif = 0
        self.iff = 0
        # self.h = 0
        # real-time data from sensor
        self.status = 'STOP'
        # self.vel = 0
        # self.height = 0
        self.real_v = 0
        self.real_h = 0

    def transferToRealValue(self,h,v):
        r_h = h / 150000.0 * 125 # mm
        r_v = v / 40.0 * 12.7 # mm/s
        r_h = round(r_h, 2)
        r_v = round(r_v, 2)
        return r_h,r_v

    def transData(self):
        self.real_h, self.real_v = self.transferToRealValue( self.h, self.v )

    def get_vel(self):
        return self.real_v
    def get_height(self):
        return self.real_h

    # encoder represent data
    def set_height(self,  height):
        self.h0 = self.h
        self.h = height
    def set_vel(self, vel):
        self.v0 = self.v
        self.v = vel
        
    def get_ab_h(self):
        return self.h
    
    def set_tar_h(self, h):
        self.hd = h
    def set_tar_v(self, v):
        self.vd = v
    
    def PID_control(self):
        kp = 1000
        ki = 10
        kd = 4
        # p 
        dif = self.h1 - self.h0
        #I
        iff = self.iff + dif
        #D
        dff = dif - self.dif        
        val  =  kp * dif + ki*iff + kd * dff
        
        self.dif = dif
        self.iff = iff
        return val
    
    def fixed_update(self,  hd,  vd):
        h = hd - 1800
        v = vd - 199900
        # self.h += h
        return h, v
            
    def update(self,  hd,  vd):
        h = hd / 125.0 * 150000
        v = vd/12.7*40
        # self.h += h
        return int(h), int(v)
