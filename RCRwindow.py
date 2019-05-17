#!/usr/bin/venv python
# -*- coding: utf-8 -*-

"""
Module implementing MainWindow.
"""
#project path
from PROJECTPATH import *
import time
# pyqt5 class
from PyQt5.QtCore import pyqtSlot, Qt
from PyQt5.QtWidgets import QMainWindow, QMessageBox,QStatusBar
from PyQt5 import QtGui
from PyQt5.QtGui import QIntValidator,QDoubleValidator,QRegExpValidator
from Ui_RCRwindow import Ui_MainWindow
# define class
from camPort import *
from cam_frame import *
from sensor_frame import *
from controlBoard_frame import *
from ur_move import *
from ros_op import roscore,roslaunch,close_roscore,close_roslaunch
from ros_node import rcrnodeThread



class MainWindow(QMainWindow, Ui_MainWindow):
    """
    Class documentation goes here.
    """

    def __init__(self, parent=None):
        """
        Constructor

        @param parent reference to the parent widget
        @type QWidget
        """
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.setWindowTitle('Single Rope Climbing Robot Control Panel')
        self.init_show()
        # camera thread
        baseCam = BaseCamera()
        self.baseCamThread = CamThread(baseCam, self.cam_base_label)
        self.baseCamThread.camSignal.connect(self.baseCamThread.change_label_pic)
        headCam = HeadCamera()
        self.headCamThread = CamThread(headCam, self.cam_head_label)
        self.headCamThread.camSignal.connect(self.headCamThread.change_label_pic)

        # sensor thread
        self.sensor1 = SensorThread()
        self.sensor1.sensorSignal.connect(self.update_sensor_data)
        self.sensor1.start()

        # control board port
        self.control1 = controlThread()
        self.control1.controlSignal.connect(self.update_rcr_data)
        self.control1.start()

        #RCR ros node
        self.rcrnode = rcrnodeThread(self.sensor1.sensor, self.control1.device.rcr)
        self.rcrnode.start()

        # self.sensor1 = SensorThread()
        # self.sensor1.sensorSignal.connect(self.update_sensor_data)
        # self.sensor1.start()

        # UR
        self.ur = UR(1)

    def init_show(self):
        self.set_rcr_btns_bool(False)
        self.set_ur_related_btns_bool(False)
        self.set_roslaunch_btn(False)
        self.set_roscore_btn(False)

        cuhk_pix = QPixmap(Project_Path + 'img/cuhk.png')
        self.icon_cuhk_label.setPixmap(cuhk_pix)
        self.icon_cuhk_label.setScaledContents(True)
        self.icon_cuhk_label.setAlignment(Qt.AlignVCenter | Qt.AlignHCenter)

        curi_pix = QPixmap(Project_Path + 'img/curi.jpeg')
        self.icon_curi_label.setPixmap(curi_pix)
        self.icon_curi_label.setScaledContents(True)
        self.icon_curi_label.setAlignment(Qt.AlignVCenter | Qt.AlignHCenter)

        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(Project_Path +'img/icon.ico'), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.setWindowIcon(icon)
        # self.setWindowIcon(QtGui.QIcon(Project_Path + "img/icon.ico"))
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)

        # txt validator
        pIntvalidator = QIntValidator()
        pIntvalidator.setRange(0, 640)
        self.cam_processing_len_edit.setValidator(pIntvalidator)

        pDoubleValidator = QDoubleValidator()
        pDoubleValidator.setRange(0, 30)
        self.cam_processing_width_edit.setValidator(QDoubleValidator())

        pIntvalidator = QIntValidator()
        pIntvalidator.setRange(0, 5000)
        self.vel_set_rcr_lineEdit.setValidator(pIntvalidator)

        pIntvalidator = QIntValidator()
        pIntvalidator.setRange(0, 50000)
        self.height_set_rcr_lineEdit.setValidator(pIntvalidator)

    """
    base cam part
    """

    @pyqtSlot()
    def on_open_cam_base_btn_clicked(self):
        """
        open base cam
        """
        port = self.com_cam_base_box.currentText()
        if not port == "":
            port = "/dev/" + port
            print(port)
            self.baseCamThread.cam.initialize_cv(port)
        else:
            self.baseCamThread.cam.initialize_cv(0)
        self.baseCamThread.resume()
        self.baseCamThread.start()
        self.open_cam_base_btn.setEnabled(False)
        self.close_cam_base_btn.setEnabled(True)

    @pyqtSlot()
    def on_close_cam_base_btn_clicked(self):
        """
        close base cam
        """
        self.baseCamThread.pause()
        self.baseCamThread.cam.close_camera()
        self.open_cam_base_btn.setEnabled(True)
        self.close_cam_base_btn.setEnabled(False)

    """
    head cam part
    """

    @pyqtSlot()
    def on_open_cam_head_btn_clicked(self):
        """
        open head cam
        """
        port = self.com_cam_head_box.currentText()
        if not port == "":
            port = "/dev/" + port
            print(port)
            self.headCamThread.cam.initialize_cv(port)
        else:
            self.headCamThread.cam.initialize_cv(0)
        self.headCamThread.resume()
        self.headCamThread.start()
        self.open_cam_head_btn.setEnabled(False)
        self.close_cam_head_btn.setEnabled(True)

    @pyqtSlot()
    def on_close_cam_head_btn_clicked(self):
        """
        close head cam
        """
        self.headCamThread.pause()
        self.headCamThread.cam.close_camera()
        self.open_cam_head_btn.setEnabled(True)
        self.close_cam_head_btn.setEnabled(False)

    """
    control board part
    """

    @pyqtSlot()
    def on_detect_control_com_btn_clicked(self):
        """
        detect control com
        """
        # 1, detect port, init  port dict
        self.control1.device.detect_port()
        # 2, clear all port box
        self.com_control_box.clear()
        # 3, show the insert port com in the port dict
        for key, value in self.control1.device.ComDict.items():
            self.com_control_box.addItem(key)
        self.open_control_com_btn.setEnabled(True)

    @pyqtSlot()
    def on_open_control_com_btn_clicked(self):
        """
        open control board
        """
        if not self.com_control_box.currentText() == "":
            port = self.com_control_box.currentText()
            self.control1.device.init(port, 19200)
            # self.sensor1.sensor.initialize_sensor(port, 9600)
            self.control1.resume()
            self.open_control_com_btn.setEnabled(False)
            self.close_control_com_btn.setEnabled(True)
            self.set_rcr_btns_bool(True)
            self.set_status_txt("openning Control board port " + port)
            # self.rcr_info_textEdit.append()
        else:
            QMessageBox.information(self,
                                    "Warning",
                                    "No port detected!",
                                    QMessageBox.Yes | QMessageBox.No)

    @pyqtSlot()
    def on_close_control_com_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        self.control1.device.close_port()
        # self.sensor1.sensor.close_port()
        self.control1.pause()
        self.open_control_com_btn.setEnabled(True)
        self.close_control_com_btn.setEnabled(False)
        self.set_rcr_btns_bool(False)
        self.set_status_txt("closing control board ")

    """
    sensor part
    """
    @pyqtSlot()
    def on_clear_sensor_txt_btn_clicked(self):
        """
        clear sensor info txt and boxes
        """
        self.sensor_info_textEdit.clear()
        self.sonic_lineEdit.clear()
        self.force_lineEdit.clear()

    @pyqtSlot()
    def on_detect_sensor_com_btn_clicked(self):
        """
        detect sensor com
        """
        # 1, detect port, init  port dict
        self.sensor1.sensor.detect_port()
        # 2, clear all port box
        self.com_sensor_box.clear()
        # 3, show the insert port com in the port dict
        for key, value in self.sensor1.sensor.ComDict.items():
            self.com_sensor_box.addItem(key)
        self.open_sensor_com_btn.setEnabled(True)

    @pyqtSlot()
    def on_open_sensor_com_btn_clicked(self):
        """
        open sensor com, make sure to give permission to the COM port
        """
        if not self.com_sensor_box.currentText() == "":
            #            self.ser.port = self.video_com_box.currentText()
            port = self.com_sensor_box.currentText()
            self.sensor1.sensor.initialize_sensor(port, 9600)
            self.sensor1.resume()
            self.open_sensor_com_btn.setEnabled(False)
            self.close_sensor_com_btn.setEnabled(True)
            self.set_status_txt("openning sensor board port " + port)
        else:
            QMessageBox.information(self,
                                    "Warning",
                                    "No port detected, pls check your port and open again!",
                                    QMessageBox.Yes | QMessageBox.No)

    def update_sensor_data(self):
        #        force = self.
        sonic = self.sensor1.sensor.get_sonic()
        force = self.sensor1.sensor.get_force()
        self.sonic_lineEdit.setText(str(sonic))
        self.force_lineEdit.setText(str(force))
        txt = "sonic: " + str(sonic) + ", " + "force: " + str(force)
        self.sensor_info_textEdit.append(txt)

    def update_rcr_data(self):
        #        force = self.
        h = self.control1.device.rcr.get_height()
        v = self.control1.device.rcr.get_vel()
        self.height_rcr_lineEdit.setText(str(h))
        self.speed_rcr_lineEdit.setText(str(v))
        txt = "height: " + str(h) + ", " + "vel: " + str(v)
        self.rcr_info_textEdit.append(txt)

    @pyqtSlot()
    def on_close_sensor_com_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        self.sensor1.sensor.close_port()
        self.sensor1.pause()
        self.open_sensor_com_btn.setEnabled(True)
        self.close_sensor_com_btn.setEnabled(False)
        self.set_status_txt("closing sensor board ")

    def set_status_txt(self, status):
        self.real_time_info_textEdit.append('The RCR is ' + status + ' now!')

    def set_rcr_btns_bool(self, bool):
        self.start_clean_ops_btn.setEnabled(bool)
        self.stop_clean_ops_btn.setEnabled(bool)
        self.open_water_pump_btn.setEnabled(bool)
        self.close_water_pump_btn.setEnabled(bool)
        self.open_sewage_pump_btn.setEnabled(bool)
        self.close_sewage_pump_btn.setEnabled(bool)

        #        self.ok_rcr_set_btn.setEnabled(bool)
        self.stop_rcr_btn.setEnabled(bool)
        self.step_up_rcr_btn.setEnabled(bool)
        self.up_rcr_btn.setEnabled(bool)
        self.down_rcr_btn.setEnabled(bool)
        self.step_down_rcr_btn.setEnabled(bool)

        self.cam_base_back_angle_btn.setEnabled(bool)
        self.cam_base_adjust_btn.setEnabled(bool)


    def set_ur_related_btns_bool(self, bool):
        self.unit_touch_window_btn.setEnabled(bool)
        self.unit_back_init_btn.setEnabled(bool)
        self.unit_auto_clean_btn.setEnabled(bool)
    #        self.cam_base_pitch_hSlider.setTracking(bool)
    #        self.cam_base_yaw_hSlider.setTracking(bool)
    """
    clean ops
    """
    @pyqtSlot()
    def on_start_clean_ops_btn_clicked(self):
        """
        clean unit rotates
        """
        self.clean_status = 'clean'
        self.set_status_txt("cleaning the window")
        self.control1.device.cmd_clean(self.clean_status)

    @pyqtSlot()
    def on_stop_clean_ops_btn_clicked(self):
        """
        clean unit stops
        """
        self.clean_status = 'cleanstop'
        self.set_status_txt("stopping cleaning the window")
        self.control1.device.cmd_clean(self.clean_status)

    @pyqtSlot()
    def on_open_water_pump_btn_clicked(self):
        """
        clean pump works
        """
        self.clean_status = 'pumpcleanrotate'
        self.set_status_txt("with clean water pumping out")
        self.control1.device.cmd_clean(self.clean_status)

    @pyqtSlot()
    def on_close_water_pump_btn_clicked(self):
        """
        clean pump stops
        """
        self.clean_status = 'pumpcleanstop'
        self.set_status_txt("with clean water STOP pumping out")
        self.control1.device.cmd_clean(self.clean_status)

    @pyqtSlot()
    def on_close_sewage_pump_btn_clicked(self):
        """
        sewage pump works
        """
        self.clean_status = 'pumpsewagestop'
        self.set_status_txt("stopping sewage water collecting")
        self.control1.device.cmd_clean(self.clean_status)

    @pyqtSlot()
    def on_open_sewage_pump_btn_clicked(self):
        """
        sewage pump stops
        """
        self.clean_status = 'pumpsewagerotate'
        self.set_status_txt("with sewage water collecting back")
        self.control1.device.cmd_clean(self.clean_status)

    @pyqtSlot()
    def on_unit_touch_window_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        self.ur_status = "urmove"
        pub = self.ur.pub
        qD = self.ur.get_final_q()
        q =getpi(qD)
        self.ur.ur_move_to_point(pub, q)
        self.set_status_txt("going to touch the window")

    @pyqtSlot()
    def on_unit_back_init_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        self.ur_status = "urback"
        pub = self.ur.pub
        qD = self.ur.get_init_q()
        q = getpi(qD)
        self.ur.ur_move_to_point(pub, q)
        self.set_status_txt("back to init joint position")

    @pyqtSlot()
    def on_unit_auto_clean_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        pub = self.ur.pub
        q = self.ur.get_final_q()
        self.ur.ur_path_move(pub, q)
        self.set_status_txt("stopping, while ur is operating")

    def set_rcr_txt(self, height, vel, dir):
        self.rcr_info_textEdit.append(dir + '; target height: ' + str(height) + ", target vel: " + str(vel))

    # def set_ab_height(self):
    #     h = self.control1.device.rcr.get_ab_h()
        # self.height_ab_rcr_lineEdit.setText(str(h))

    @pyqtSlot()
    def on_stop_rcr_btn_clicked(self):
        """
        RCR Emergency stops
        """
        self.status = 'stop'
        self.control1.device.car_status = "STOP"
        self.set_status_txt(self.status)
        self.control1.device.cmd_move(self.status)
        self.statusBar.showMessage("Emergency stop cars!!!", 2000)
        # self.set_ab_height()

    @pyqtSlot()
    def on_down_rcr_btn_clicked(self):
        """
        RCR down step moves fixed height, fixed speed
        """
        self.status = 'down'
        height = self.height_set_rcr_lineEdit.text()
        vel = self.vel_set_rcr_lineEdit.text()
        #        self.speed_rcr_hSlider.setValue(vel)
        #        self.height_rcr_hSlider.setValue(height)
        height = -float(height)
        vel = float(vel)
        direction = "DOWN"
        self.control1.device.cmd_precise_move(height, vel)
        self.set_rcr_txt(height, vel, direction)
        self.set_status_txt(self.status)
        # self.set_ab_height()

    @pyqtSlot()
    def on_up_rcr_btn_clicked(self):
        """
        RCR up step moves, fixed height, fixed speed
        """
        self.status = 'up'
        height = self.height_set_rcr_lineEdit.text()
        vel = self.vel_set_rcr_lineEdit.text()
        height = float(height)
        vel = float(vel)
        self.control1.device.cmd_precise_move(height, vel)
        direction = "UP"
        #        txt = "given target: " + str(height) + ", "+  "vel: " + str(vel)
        self.set_rcr_txt(height, vel, direction)
        self.set_status_txt(self.status)
        # self.set_ab_height()

    @pyqtSlot()
    def on_step_up_rcr_btn_clicked(self):
        """
        when btn pressed, rcr keeps up
        """
        self.status = 'up'
        status_txt = "step " + self.status
        self.set_status_txt(status_txt)
        self.control1.device.cmd_move(self.status)
        # self.set_ab_height()

    @pyqtSlot()
    def on_step_down_rcr_btn_clicked(self):
        """
        when btn released, rcr stops
        """
        self.status = 'down'
        status_txt = "step "+ self.status
        self.set_status_txt(status_txt)
        self.control1.device.cmd_move(self.status)
        # self.set_ab_height()

    @pyqtSlot()
    def on_cam_base_adjust_btn_clicked(self):
        """
        adjust cam servo motors
        """
        pitch = self.cam_base_pitch_hSlider.value()
        yaw = self.cam_base_yaw_hSlider.value()
        len = self.cam_processing_len_edit.text()
        wid = self.cam_processing_width_edit.text()
        self.baseCamThread.cam.set_cam_parameters(int(len),float(wid))
        pitch, yaw = self.control1.device.cmd_cam_adjust(pitch, yaw)
        status = "goint to angles as, pitch: " + str(pitch) + ", yaw: " + str(yaw)
        self.cam_set_status_txt(status)

    @pyqtSlot()
    def on_cam_base_param_adjust_btn_clicked(self):
        """
        adjust cam servo motors
        """
        len = self.cam_processing_len_edit.text()
        wid = self.cam_processing_width_edit.text()
        self.baseCamThread.cam.set_cam_parameters(int(len), float(wid))
        # pitch, yaw = self.control1.device.cmd_cam_adjust(pitch, yaw)
        status = "setting base cam param as: length " + str(len) + ", width: " + str(wid)
        self.cam_set_status_txt(status)

    @pyqtSlot(int)
    def on_cam_base_pitch_hSlider_valueChanged(self, value):
        """
        yaw value changed
        @param value DESCRIPTION
        @type int
        """
        self.cam_base_pitch_ledit.setText(str(100 + value))

    @pyqtSlot(int)
    def on_cam_base_yaw_hSlider_valueChanged(self, value):
        """
        yaw value changed
        @param value DESCRIPTION
        @type int
        """
        self.cam_base_yaw_ledit.setText(str(50 + value))

    @pyqtSlot()
    def on_cam_base_back_angle_btn_clicked(self):
        """
        back to the init base cam angle
        """
        pitch, yaw = self.control1.device.get_basecam_angles()
        self.cam_base_pitch_hSlider.setValue(pitch)
        self.cam_base_yaw_hSlider.setValue(yaw)
        self.control1.device.go_init_basecam_angle()
        status = "goint to INIT angles as, pitch: " + str(pitch) + ", yaw: " + str(yaw)
        self.cam_set_status_txt(status)

    @pyqtSlot()
    def on_cam_base_set_angle_btn_clicked(self):
        """
        set the init base cam angle
        """
        pitch = self.cam_base_pitch_hSlider.value()
        yaw = self.cam_base_yaw_hSlider.value()
        pitch, yaw = self.control1.device.set_init_basecam_angle(pitch, yaw)
        status = "set INIT angles as, pitch: " + str(pitch) + ", yaw: " + str(yaw)
        self.cam_set_status_txt(status)

    def cam_set_status_txt(self, status):
        self.real_time_info_textEdit.append('The base cam is ' + status + ' now!')

    @pyqtSlot()
    def on_clear_rcr_txt_btn_clicked(self):
        """
        clear rcr info txt and boxes
        """
        self.rcr_info_textEdit.clear()

    @pyqtSlot()
    def on_clear_real_time_txt_btn_clicked(self):
        """
        clear real time info txt
        """
        self.real_time_info_textEdit.clear()

    """
    UR part
    """
    def set_ur_info_txt(self, txt):
        self.ur_info_textEdit.append(txt)

    @pyqtSlot()
    def on_clear_ur_info_txt_btn_clicked(self):
        self.ur_info_textEdit.clear()
    
    @pyqtSlot()
    def on_ur_choose_ok_btn_clicked(self):
        """set ur type and init the kinematics"""
        ur_type = self.ur_choose_box.currentText()
        self.ur.set_UR_ROBOT(ur_type)
        self.set_ur_info_txt("set UR type: " + ur_type )

    def set_roslaunch_btn(self, bool):
        self.ur_close_launch_btn.setEnabled(bool)
        self.ur_launch_btn.setEnabled(not bool)

    def set_roscore_btn(self, bool):
        self.ur_close_core_btn.setEnabled(bool)
        self.ur_core_btn.setEnabled(not bool)

    @pyqtSlot()
    def on_ur_launch_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        ur_type = self.ur.urtype
        # print(ur_type)
        roslaunch(ur_type)
        self.set_ur_info_txt("UR type: " + ur_type +", launch UR...")
        self.ur.Init_node(ur_type)
        self.set_ur_eepos_btns_bool(True)
        self.set_roslaunch_btn(True)
        self.set_ur_related_btns_bool(True)
        # self.ur_launch_btn()

    @pyqtSlot()
    def on_ur_close_launch_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        ur_type = self.ur.urtype
        # print(ur_type)
        close_roslaunch(ur_type)
        self.set_ur_info_txt("close launch " + ur_type )
        # self.ur.Init_node(ur_type)
        self.set_ur_eepos_btns_bool(False)
        self.set_roslaunch_btn(False)
        self.set_ur_related_btns_bool(False)
        # self.ur_launch_btn()

    @pyqtSlot()
    def on_ur_core_btn_clicked(self):
        """
        Slot documentation goes here.
        """

        roscore()
        time.sleep(0.5)
        # import os
        # cmd = "sh " + Project_Path + "shell/roscore.sh"
        # os.system(cmd)
        # os.system("sh shell/roscore.sh")
        self.set_ur_info_txt("open ros.")
        self.set_roscore_btn(True)

    @pyqtSlot()
    def on_ur_close_core_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        close_roscore()
        self.set_ur_info_txt("close ros.")
        self.set_roscore_btn(False)


    @pyqtSlot()
    def on_rcrnode_open_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        # self.rcrnode.init_node()
        self.rcrnode.resume()

    @pyqtSlot()
    def on_get_init_ur_pos_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        q_list = self.ur.set_init_q()
        q = self.ur.get_init_q()
        stri = "set INITT ur pos:" + '[' + ','.join(map(str,q)) + '].'
        self.set_ur_info_txt(stri)
        msg = QMessageBox.information(self,
                                "Confirm",
                                "Pls confirm your pos choosen!",
                                QMessageBox.Yes | QMessageBox.No)
        if (msg == QMessageBox.Yes):
            self.ur.ur_init_ready = 1
            self.set_ur_info_txt(" init pos is ok!.")
            # if self.ur.ur_final_ready == 1:
            #     self.ur.ur_ready = 1
        else:
            self.ur.ur_init_ready = 0
            self.ur.ur_ready = 0
        self.ur.ur_ready = self.ur.ur_init_ready * self.ur.ur_final_ready
        self.set_ur_related_btns_bool(self.ur.ur_ready)


    @pyqtSlot()
    def on_get_final_ur_pos_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        q_list = self.ur.set_final_q()
        q = self.ur.get_final_q()

        stri = "set FINAL ur pos:" + '[' + ','.join(map(str,q)) + '].'
        self.set_ur_info_txt(stri)
        msg = QMessageBox.information(self,
                                "Confirm",
                                "Pls confirm your pos choosen!",
                                QMessageBox.Yes | QMessageBox.No)
        if (msg == QMessageBox.Yes):
            self.ur.ur_final_ready = 1
            self.set_ur_info_txt(" final pos is ok!.")
            # if self.ur.ur_init_ready == 1:
            #     self.ur.ur_ready = 1
        else:
            self.ur.ur_init_ready = 0
            self.ur.ur_ready = 0
        self.ur.ur_ready = self.ur.ur_init_ready * self.ur.ur_final_ready
        self.set_ur_related_btns_bool(self.ur.ur_ready)

    """UR EE Pos step movement"""
    def set_ur_eepos_btns_bool(self, bool):
        self.ur_forward_btn.setEnabled(bool)
        self.ur_back_btn.setEnabled(bool)
        self.ur_up_btn.setEnabled(bool)
        self.ur_down_btn.setEnabled(bool)
        self.ur_left_btn.setEnabled(bool)
        self.ur_right_btn.setEnabled(bool)

    @pyqtSlot()
    def on_ur_forward_btn_clicked(self):
        q = self.ur.get_q()
        pub = self.ur.pub
        self.ur.ur_step_move_forward(q,pub)
    @pyqtSlot()
    def on_ur_back_btn_clicked(self):
        q = self.ur.get_q()
        pub = self.ur.pub
        self.ur.ur_step_move_backward(q, pub)
    @pyqtSlot()
    def on_ur_up_btn_clicked(self):
        q = self.ur.get_q()
        pub = self.ur.pub
        self.ur.ur_step_move_up(q,pub)
    @pyqtSlot()
    def on_ur_down_btn_clicked(self):
        q = self.ur.get_q()
        pub = self.ur.pub
        self.ur.ur_step_move_down(q, pub)
    @pyqtSlot()
    def on_ur_left_btn_clicked(self):
        q = self.ur.get_q()
        pub = self.ur.pub
        self.ur.ur_step_move_left(q, pub)
    @pyqtSlot()
    def on_ur_right_btn_clicked(self):
        q = self.ur.get_q()
        pub = self.ur.pub
        self.ur.ur_step_move_right(q, pub)
