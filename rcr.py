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
        self.hd = 0
        self.vd = 0
        self.h0 = 0
        self.v0 = 0
        self.h1 = 0
        self.v1 = 0
        self.dif = 0
        self.iff = 0
        self.h = 0
        self.status = 'STOP'        
        
    def get_vel(self):
        return self.vel        
    def get_height(self):
        return self.height        
    def set_height(self,  height):
        self.height = height
    def set_vel(self, vel):
        self.vel = vel
        
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
        self.h += h
        return h, v
            
    def update(self,  hd,  vd):
        h = hd + 1800
        v = vd+199900
        self.h += h
        return h, v
