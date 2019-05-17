#!/usr/bin/venv python
# -*- coding: utf-8 -*-

# Copyright (c) 2018.10, Guangli SUN
# All rights reserved.

import rospy,time

from ur3_kinematics import *
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import os

class UR():

    def __init__(self, num):
#        self.port = ''
        self.qstart = [ 91.48, -202.73, 70.01, 136.88, -88.94, 180.24 ]
        self.ur_num = num
        self.q = self.qstart
        self.ur_status = 'urstop'
        self.weights = [1.] * 6
        self.radius = 0.22
        # weights = [1.] * 6
        self.cont = 15
        self.now_ur_pos = self.qstart
        self.init_q = self.qstart
        self.final_q = self.init_q
        self.ur_init_ready = 0
        self.ur_final_ready = 0
        self.ur_ready = 0
        self.kin = Kinematic()
        self.STEP = 0.002 # unit: m
        self.urtype = "ur3"
        rospy.init_node("move_ur_circle")

    """kinematics related """
    def set_UR_ROBOT(self,type):
        self.kin.set_urtype(type)
        self.urtype = type
    
    def get_urobject_urkine(self):
        ur0 = Kinematic()
        return ur0

    def get_IK_from_T(self,T,q_last):
        ur0 = self.kin
        return ur0.best_sol( self.weights, q_last, T )

    def solve_q(self, q, T):
        # rad
        qd = self.kin.best_sol(self.weights, q, T)
        return qd

    """ros related """
    def Init_node(self,type="ur3"):
        # rospy.init_node("move_ur_circle")
        self.pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)
        self.sub = rospy.Subscriber("/joint_states", JointState, self.callback)
        return self.pub,  self.sub

    def callback(self, msg):
        self.read_pos_from_ur_joint( msg )
#        self.ur_pose_buff_list, self.ave_ur_pose = self.pos_filter_ur( self.ur_pose_buff_list, self.now_ur_pos )

    def read_pos_from_ur_joint(self, msg):
        self.now_ur_pos = list( msg.position )
  
    """q related """
    def get_qstart(self):
        return self.qstart
    
    def get_q(self):
        return self.now_ur_pos
    
    def get_T(self):
        return self.kin.Forward(self.now_ur_pos)
    
    def get_init_q(self):
        q = getDegree( self.init_q )
        return q
    def set_init_q(self):
        self.init_q = self.now_ur_pos
        return self.init_q
        
    def get_final_q(self):
        q = getDegree(self.final_q)
        return q
    def set_final_q(self):
        self.final_q = self.now_ur_pos
        return self.final_q

    """ element ur move """
    # q should be in rad unit, if not use getpi(q) to transfer it
    def ur_move_to_point(self, pub, q ):
        t,vel,ace = self.set_ur_ros_params()
        # q_in = getpi(q)
        self.urscript_pub( pub, q, vel, ace, t )

    def ur_step_move(self, q, pub, i, direction):
        # t,vel,ace = self.set_ur_ros_params()
        urk = self.kin
        F_T = urk.Forward(q)
        # TransT = self.get_T_translation(F_T)
        F_T[i] = F_T[i] + direction*self.STEP  
        # qd = urk.best_sol(self.weights, q, F_T)
        # sometimes there are negative and positive difference, using weights to revise
        qd = self.solve_q(q, F_T)
        self.ur_move_to_point(pub,qd)
        # self.urscript_pub( pub, qd, vel, ace, t )

    def ur_step_move_up(self, q, pub):
        self.ur_step_move(q,pub, 11, -1)
    
    def ur_step_move_down(self, q, pub):
        self.ur_step_move(q,pub, 11, 1)
    
    def ur_step_move_left(self, q, pub):
        self.ur_step_move(q,pub, 3, -1)
    
    def ur_step_move_right(self, q, pub):
        self.ur_step_move(q,pub, 3, 1)
    
    def ur_step_move_forward(self, q, pub):
        self.ur_step_move(q,pub, 7, -1)
    
    def ur_step_move_backward(self, q, pub):
        self.ur_step_move(q,pub, 7, 1)
    
    def set_ur_ros_params(self):
        self.t = 0
        self.vel = 0.2
        self.ace = 50
        return self.t, self.vel, self.ace
    
    def get_T_translation(self, T):
        trans_x = T[3]
        trans_y = T[7]
        trans_z = T[11]
        return [trans_x, trans_y, trans_z]

    """ path ur move """
    def get_draw_line_xy(self,t,xy_center_pos,flag):
        if flag==0:
            # y = xy_center_pos[1] + self.radius * t / self.cont
            # x = xy_center_pos[0]
            # x = xy_center_pos[0] + self.radius  * t / self.cont
            # y = xy_center_pos[1]
            x = xy_center_pos[1] + self.radius * t / self.cont
            y = xy_center_pos[0]
            return  [x,y]
        elif flag==1:
            # y = xy_center_pos[1] - self.radius * t / self.cont
            # x = xy_center_pos[0]
            # x = xy_center_pos[0] - self.radius  * t / self.cont
            # y = xy_center_pos[1]
            x = xy_center_pos[1] - self.radius * t / self.cont
            y = xy_center_pos[0]
            return  [x,y]
        else:
            pass

    def get_q_list(self,T_list,qzero):
        try:
            ur0 = self.kin
            tempq=[]
            resultq=[]
            for i in range(len(T_list)):
                if i==0:
                    tempq=qzero
                    firstq = ur0.best_sol(self.weights, tempq, T_list[i])
                    tempq=firstq
                    resultq.append(firstq.tolist())
                    # print "firstq", firstq
                else:
                    qq = ur0.best_sol(self.weights, tempq, T_list[i])
                    tempq=qq
                    # print "num i qq",i,qq
                    resultq.append(tempq.tolist())
            return resultq
        except:
            print("ur kinematics error")

    
    def insert_new_xy(self,T,nx,ny):
        temp=[]
        for i in range(12):
            if i==3:
                temp.append(nx)
            elif i==7:
                temp.append(ny)
            else:
                temp.append(T[i])
        return temp

    def get_line_T(self,InitiT,xy_center_pos):
        left=[]
        right=[]
        for i in range(2*self.cont):
            # new_xy=self.get_draw_circle_xy(i,xy_center_pos)
            if i< self.cont:
                new_xy = self.get_draw_line_xy(i, xy_center_pos,0)
                print("new_xy_left------------------", new_xy)
                new_T=self.insert_new_xy(InitiT,new_xy[0],new_xy[1])
                #print "initial T\n",InitiT
                #print "new_T\n",i,new_T
                left.append(new_T)
            elif i>=self.cont:
                new_xy = self.get_draw_line_xy((i-self.cont), xy_center_pos,1)
                print("new_xy_right------------------", new_xy)
                new_T=self.insert_new_xy(InitiT,new_xy[0],new_xy[1])
                #print "initial T\n",InitiT
                #print "new_T\n",i,new_T
                right.append(new_T)
        return left,right
    
    def __str__(self):
        print("ur move")
        
    def ur_path_move(self, pub, q):
        ratet=3
        t = 0
        vel = 0.1
        ace = 50
        rate = rospy.Rate(ratet)
        q = getpi(q)
        urk = self.kin
        F_T = urk.Forward(q)
#        print "F_T", F_T
        TransT = self.get_T_translation(F_T)
#        print "TransT", TransT
        xy_center_pos = [TransT[0], TransT[1]]
#        print "xy_center_pos", xy_center_pos
        T_list_left, T_list_right = self.get_line_T(F_T, xy_center_pos)
        # print "T_list", T_list_left
#        print "T_list_left,T_list_right ",T_list_left,"\n----------------",T_list_right
        reslut_q_left = self.get_q_list(T_list_left, q)
        reslut_q_right = self.get_q_list(T_list_right, q)
#        print "reslut_q_right", reslut_q_right
        reslut_q_left = reslut_q_left + reslut_q_left[len(reslut_q_left) - 1::-1]
        # reslut_q = reslut_q_right+reslut_q_left
        reslut_q_right = reslut_q_right + reslut_q_right[len(reslut_q_right) - 1::-1]
        cn_left = 0
        cn_right = 0
        left_flag_0 = 0
        right_flag_0 = 0
        cnt_clean = 0

        while not rospy.is_shutdown() and cnt_clean < 4 * self.cont:
            cnt_clean += 1
            if reslut_q_left != None and reslut_q_right != None:
                if left_flag_0 == 0:
                    self.urscript_pub(pub, reslut_q_left[cn_left], vel, ace, t)
                    cn_left += 1
                    if cn_left == len(reslut_q_left):
                        cn_left = 0
                        left_flag_0 = 1
                        right_flag_0 = 1
                    time.sleep(0.009)
                if right_flag_0 == 1:
                    self.urscript_pub(pub, reslut_q_right[cn_right], vel, ace, t)
                    cn_right += 1
                    if cn_right == len(reslut_q_right):
                        cn_right = 0
                        left_flag_0 = 0
                        right_flag_0 = 0
                        turn_motor_upward = 1
#                    print "cn_right-----\n,cn_right", reslut_q_right[cn_right], cn_right
                    time.sleep(0.009)
            rate.sleep()

    def urscript_pub(self, pub, qq, vel, ace, t):
        ss = "movej([" + str(qq[0]) + "," + str(qq[1]) + "," + str(qq[2]) + "," + str(qq[3]) + "," + str(qq[4]) + "," + str(qq[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(vel) + "," + "t=" + str(t) + ")"
        print("---------ss:", ss)
        # ss="movej([-0.09577000000000001, -1.7111255555555556, 0.7485411111111111, 0.9948566666666667, 1.330836666666667, 2.3684322222222223], a=1.0, v=1.0,t=5)"
        pub.publish(ss)

def getpi(l):
    res = []
    print(l)
    for i in l:
        res.append( i / 180 * math.pi )
    return res

def getDegree(l):
    res = []
    for i in l:
        res.append(  i*180 / math.pi )
    return res

# def main():
#     pass
#
# def test():
#     ur = UR(1)

if __name__ == '__main__':
    ur = UR(1)
    ur.Init_node()
    q = ur.qstart
    print(q)
    print(getpi(q))
    ratet = 3
    rate = rospy.Rate(ratet)
    while not rospy.is_shutdown():
        ur.ur_move_to_point(ur.pub,getpi(ur.qstart))
        rospy.spinOnce()
