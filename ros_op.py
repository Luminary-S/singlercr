#!/usr/bin/python

import os

dir = "/home/sgl/catkin_new/src/singrcr/launch/"
file = "ur5_launch.launch"
fname = dir+file

def roscore():
    os.system("sh ./shell/roscore.sh")

def roslaunch():
    os.system("sh ./shell/roslaunch.sh")

def main():
    roscore()


if __name__ == '__main__':
    main()