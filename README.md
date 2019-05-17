# 1. Single RCR control panel
support ur5 and ur3, should revise:

<!-- TOC -->

- [1. Single RCR control panel](#1-single-rcr-control-panel)
- [2. To do list](#2-to-do-list)
- [process](#process)
  - [UR](#ur)
- [3. Reminder](#3-reminder)
- [4. Procedure](#4-procedure)
  - [4.1. Sensor INFO frame](#41-sensor-info-frame)
  - [4.2. Base camera and head camera](#42-base-camera-and-head-camera)
  - [4.3. *cleaning panel*, *rope climbing robot panel* and *base camera pos adjustment*](#43-cleaning-panel-rope-climbing-robot-panel-and-base-camera-pos-adjustment)
- [5. Requirements](#5-requirements)
- [6. recommend IDE : Pycharm](#6-recommend-ide--pycharm)
- [7. Hardware requirements](#7-hardware-requirements)
- [8. relation](#8-relation)
- [9. Following btns and slider can be used if ticked.](#9-following-btns-and-slider-can-be-used-if-ticked)
  - [9.1. realtime info frame](#91-realtime-info-frame)
  - [9.2. Sensor INFO frame](#92-sensor-info-frame)
  - [9.3. base camera and head camera](#93-base-camera-and-head-camera)
  - [9.4. cleaning panel](#94-cleaning-panel)
  - [9.5. RCR panel](#95-rcr-panel)
  - [9.6. UR INFO (should be done by myself, don't press any btn)](#96-ur-info-should-be-done-by-myself-dont-press-any-btn)
  - [9.7. Base camera pos adjustment](#97-base-camera-pos-adjustment)

<!-- /TOC -->

# 2. To do list
1. [x] sensor serialPort and control serialPort merge
2. [x] rcr up and down
3. [ ] position and velocity relation
4. [ ] stop the car

# process
## UR
1. roscore
2. choose ur type
3. roslaunch singrcr ur3_launch.launch
4. get init ur pos
5. get final ur pos ( this step should be calculated based on the sonic value and ur pos, use impedance control to set)
6. move up and down

# 3. Reminder
1. control Arduino baud rate: 19200
2. sensor Arduino  baud rate: 9600

# 4. Procedure
1. start "run.py" by Python3 in terminal
## 4.1. Sensor INFO frame
1. start your own serial port firstly by detecting the port
2. choose the right port in the box, then "open" it
3. "close" can be enabled when the port is open
## 4.2. Base camera and head camera
1. choose the right camera port in the box, "open" it 
2. "close" can be enabled when the port is open
## 4.3. *cleaning panel*, *rope climbing robot panel* and *base camera pos adjustment*
1. always enabled before the port is detected and chosen correctly
2. adjust the angles of pitch and yaw by sliders, values will be changed with the slider simultaneously.
3. pitch and yaw are adjusted with range 100~200 and 50~210; only in integer
4. the realisation of "stepDown, stepUP" can be seen in "rcr.py", function name is "fixed_update(self,  hd,  vd)"
5. the realisation of "Down, UP" can be seen in "rcr.py", function name is "update(self,  hd,  vd)"
6. base camera is the camera installed in the base, the pos can be adjusted by the base camera pos adjustment servo motors
7. In base camera, "lsd" method is embedded. While the head camera, just show the raw images.
8. choose the right port of the camera.

# 5. Requirements
1. IDE: Eric6 or Pycharm
2. Python 3.5
3. Opencv3+, like: Opencv3.4.6
4. ROS: Kinetic; no need now
5. PyQt5, Qt based GUI

# 6. recommend IDE : Pycharm
1. you should have a python environment in your directory.
2. you can set it by open : Setting ->project-> project interpreter -> add -> new environment based on "the python3 of your computer", select "inherit from global site-packages"
3. after setting, you will have a "venv"(environment name) folder in your directory
4. you should set your python libraries by following the [blog](https://blog.csdn.net/tterminator/article/details/79802094)
5. you should add: 
    1. "/usr/local/lib/python3.5/site-packages"  -> this is your own python3.5 libraries
    2. "/opt/ros/kinetic/lib/python2.7/site-packages" -> this has "rospy", if not "rospy error" happens
    3. in python3, you should install "rospkg, catkin_tools, catkin_pkg" and "opencv-python"
     

# 7. Hardware requirements
1. spark fun 1401 IMU
2. two industry camera(web cams), one for **RCR Base**, other for **UR EE IBVS**
3. UR3
4. two arduino: one for sensor data pre-processing, other for motors control
5. Four motors include, clean water and sewage pumps, cleaning unit rotating 12V DC motor, RCR climbing 24v DC motor.  
6. [Robomodule](http://www.robomodule.net/download.html) driver is CAN protocal based.


# 8. relation
1. 200000 = 165mm
2. cam processing parameters: len=400; width=9

# 9. Following btns and slider can be used if ticked. 
## 9.1. realtime info frame
1. [x] clear_real_time_txt_btn

## 9.2. Sensor INFO frame
1. [x] clear_sensor_txt_btn
2. [x] detect_sensor_com_btn
3. [x] com_sensor_box
4. [x] open_sensor_com_btn
5. [x] close_sensor_com_btn

## 9.3. base camera and head camera
1. [x] open_cam_base_btn
2. [x] close_cam_base_btn
3. [x] com_cam_base_box
4. [x] open_cam_head_btn
5. [x] close_cam_head_btn
6. [x] com_cam_head_box

## 9.4. cleaning panel
1. [x] start_clean_ops_btn
2. [x] stop_clean_ops_btn
3. [x] open_water_pump_btn
4. [x] close_water_pump_btn
5. [x] open_sewage_pump_btn
6. [x] close_sewage_pump_btn
7. [x] unit_touch_window_btn
8. [x] unit_back_init_btn
9. [ ] unit_auto_clean_btn

## 9.5. RCR panel
12. [x] up_rcr_set_btn
13. [x] down_rcr_btn
14. [x] step_up_rcr_btn
15. [x] step_down_rcr_btn
16. [x] clear_rcr_txt_btn
17. [x] stop_rcr_btn
18. [ ] speed_rcr_hSlider
19. [ ] height_rcr_hSlider

## 9.6. UR INFO (should be done by myself, don't press any btn)
1. [ ] ur_forward_btn
2. [ ] ur_back_btn
3. [ ] ur_left_btn
4. [ ] ur_right_btn
5. [ ] ur_up_btn
6. [ ] ur_down_btn

## 9.7. Base camera pos adjustment
1. [x] cam_base_pitch_hSlider
2. [x] cam_base_yaw_hSlider
3. [x] cam_base_pitch_btn
4. [x] cam_base_yaw_btn
5. [x] cam_base_adjust_btn
6. [x] cam_base_back_angle_btn
7. [x] cam_base_set_angle_btn
 
