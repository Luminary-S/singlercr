#!/usr/bin/venv python
# -*- coding: utf-8 -*-

# Copyright (c) 2018.10, Guangli SUN
# All rights reserved.

import rospy,time

# from ur3_kinematics import *
# from std_msgs.msg import String
from sensor_msgs.msg import JointState
import os

def callback( msg ):
    read_pos_from_ur_joint( msg )
#        self.ur_pose_buff_list, self.ave_ur_pose = self.pos_filter_ur( self.ur_pose_buff_list, self.now_ur_pos )

def read_pos_from_ur_joint(msg):
    now_ur_pos = list( msg.position )
    print(now_ur_pos)

    stri = "set INITT ur pos:" + '[' + ', '.join( [ str(i) for i in now_ur_pos] ) + '].'
    rospy.loginfo(stri)

rospy.init_node("move_ur_circle")
# pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)
sub = rospy.Subscriber("/joint_states", JointState, callback)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    # hello_str = "hello world %s" % rospy.get_time()
    # rospy.loginfo(hello_str)

    rate.sleep()
# return self.pub, self.sub