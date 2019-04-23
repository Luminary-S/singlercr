#!/usr/bin/python

import os

# dir = "/home/sgl/catkin_new/src/singrcr/launch/"
# file = "ur5_launch.launch"
# fname = dir+file
# from ur3_kinematics import UR_ROBOT

def roscore():
    os.system("sh ./shell/roscore.sh")

def roslaunch(ur_type):
    if ur_type=="ur5":
        os.system("sh ./shell/roslaunch.sh")
        # UR_ROBOT ="ur5"
    elif ur_type=="ur3":
        os.system("sh ./shell/roslaunch_ur3.sh")
        # UR_ROBOT = "ur3"

def roslaunch_ur3():
    os.system("sh ./shell/roslaunch_ur3.sh")

def main():
    roscore()


if __name__ == '__main__':
    main()