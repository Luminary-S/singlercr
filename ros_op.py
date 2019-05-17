#!/usr/bin/python
#project path
from PROJECTPATH import *
import os
import time
# dir = "/home/sgl/catkin_new/src/singrcr/launch/"
# file = "ur5_launch.launch"
# fname = dir+file
# from ur3_kinematics import UR_ROBOT

def roscore():
    cmd = "sh " + Project_Path +"shell/roscore.sh"
    os.system(cmd)
    # print("ok")

def close_roscore():
    os.system("ps -ef | grep roscore | xargs kill")

def roslaunch(ur_type):
    if ur_type=="ur3":
        cmd = "sh " + Project_Path +"shell/roslaunch_ur3.sh"
        os.system( cmd )
        # UR_ROBOT ="ur5"
    elif ur_type=="ur5":
        # print(ur_type)
        cmd = "sh " + Project_Path + "shell/roslaunch_ur5.sh"
        os.system(cmd)
        # print("ok")

def roslaunch_ur3():
    os.system("sh " + Project_Path +"shell/roslaunch_ur3.sh")

def close_roslaunch(ur_type):
    if ur_type=="ur3":
        os.system("ps -ef | grep ur3_launch | awk '{print $2}' | xargs kill -9")
        # UR_ROBOT ="ur5"
    elif ur_type=="ur5":
        os.system("ps -ef | grep ur5_launch | awk '{print $2}' | xargs kill -9")

# def roscore():
#     cmd = "sh shell/roscore.sh"
#     os.system(cmd)
#
# def close_roscore():
#     os.system("ps -ef | grep roscore | xargs kill")
#
# def roslaunch(ur_type):
#     if ur_type=="ur3":
#         cmd = "sh shell/roslaunch_ur3.sh"
#         os.system( cmd )
#         # UR_ROBOT ="ur5"
#     elif ur_type=="ur5":
#         # print(ur_type)
#         cmd = "sh shell/roslaunch_ur5.sh"
#         os.system(cmd)
#         # print("ok")
#
# def roslaunch_ur3():
#     os.system("sh shell/roslaunch_ur3.sh")
#
# def close_roslaunch(ur_type):
#     if ur_type=="ur3":
#         os.system("ps -ef | grep ur3_launch | xargs kill")
#         # UR_ROBOT ="ur5"
#     elif ur_type=="ur5":
#         os.system("ps -ef | grep ur5_launch | xargs kill")

if __name__ == '__main__':
    roscore()
    time.sleep(1)
    roslaunch("ur3")