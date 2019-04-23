#!/usr/bin/venv python
# -*- coding: utf-8 -*-

"""
Module implementing MainWindow.
"""
# pyqt5 class
from PyQt5.QtCore import pyqtSlot, Qt
from PyQt5.QtWidgets import QMainWindow, QMessageBox
from PyQt5 import QtGui

from Ui_RCRwindow import Ui_MainWindow
# define class
from camPort import *
from cam_frame import *
from sensor_frame import *
from controlBoard_frame import *
from ur_move import *


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
        self.controlBoard = ControlBoard()

        # UR
        self.ur = UR(1)

    def init_show(self):
        self.set_rcr_btns_bool(False)
        cuhk_pix = QPixmap('img/cuhk.png')
        self.icon_cuhk_label.setPixmap(cuhk_pix)
        self.icon_cuhk_label.setScaledContents(True)
        self.icon_cuhk_label.setAlignment(Qt.AlignVCenter | Qt.AlignHCenter)

        curi_pix = QPixmap('img/curi.jpeg')
        self.icon_curi_label.setPixmap(curi_pix)
        self.icon_curi_label.setScaledContents(True)
        self.icon_curi_label.setAlignment(Qt.AlignVCenter | Qt.AlignHCenter)

        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("img/icon.ico"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.setWindowIcon(icon)

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
        self.controlBoard.detect_port()
        # 2, clear all port box
        self.com_control_box.clear()
        # 3, show the insert port com in the port dict
        for key, value in self.controlBoard.ComDict.items():
            self.com_control_box.addItem(key)
        self.open_control_com_btn.setEnabled(True)

    @pyqtSlot()
    def on_open_control_com_btn_clicked(self):
        """
        open control board
        """
        if not self.com_control_box.currentText() == "":
            port = self.com_control_box.currentText()
            self.controlBoard.init(port, 115200)
            self.open_control_com_btn.setEnabled(False)
            self.close_control_com_btn.setEnabled(True)
            self.set_rcr_btns_bool(True)
            self.set_status_txt("openning port " + port)
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
        self.controlBoard.close_port()
        self.open_control_com_btn.setEnabled(True)
        self.close_control_com_btn.setEnabled(False)
        self.set_rcr_btns_bool(False)
        self.set_status_txt("closed")

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

    @pyqtSlot()
    def on_close_sensor_com_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        self.sensor1.sensor.close_port()
        self.sensor1.pause()
        self.open_sensor_com_btn.setEnabled(True)
        self.close_sensor_com_btn.setEnabled(False)

    def set_status_txt(self, status):
        self.real_time_info_textEdit.append('The RCR is ' + status + ' now!')

    def set_rcr_btns_bool(self, bool):
        self.start_clean_ops_btn.setEnabled(bool)
        self.stop_clean_ops_btn.setEnabled(bool)
        self.open_water_pump_btn.setEnabled(bool)
        self.close_water_pump_btn.setEnabled(bool)
        self.open_sewage_pump_btn.setEnabled(bool)
        self.close_sewage_pump_btn.setEnabled(bool)
        self.unit_touch_window_btn.setEnabled(bool)
        self.unit_back_init_btn.setEnabled(bool)
        self.unit_auto_clean_btn.setEnabled(bool)
        #        self.ok_rcr_set_btn.setEnabled(bool)
        self.stop_rcr_btn.setEnabled(bool)
        self.step_up_rcr_btn.setEnabled(bool)
        self.up_rcr_btn.setEnabled(bool)
        self.down_rcr_btn.setEnabled(bool)
        self.step_down_rcr_btn.setEnabled(bool)

        self.cam_base_back_angle_btn.setEnabled(bool)
        self.cam_base_adjust_btn.setEnabled(bool)

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
        self.controlBoard.cmd_clean(self.clean_status)

    @pyqtSlot()
    def on_stop_clean_ops_btn_clicked(self):
        """
        clean unit stops
        """
        self.clean_status = 'cleanstop'
        self.set_status_txt("stopping cleaning the window")
        self.controlBoard.cmd_clean(self.clean_status)

    @pyqtSlot()
    def on_open_water_pump_btn_clicked(self):
        """
        clean pump works
        """
        self.clean_status = 'pumpcleanrotate'
        self.set_status_txt("with clean water pumping out")
        self.controlBoard.cmd_clean(self.clean_status)

    @pyqtSlot()
    def on_close_water_pump_btn_clicked(self):
        """
        clean pump stops
        """
        self.clean_status = 'pumpcleanstop'
        self.set_status_txt("with clean water STOP pumping out")
        self.controlBoard.cmd_clean(self.clean_status)

    @pyqtSlot()
    def on_close_sewage_pump_btn_clicked(self):
        """
        sewage pump works
        """
        self.clean_status = 'pumpsewagestop'
        self.set_status_txt("stopping sewage water collecting")
        self.controlBoard.cmd_clean(self.clean_status)

    @pyqtSlot()
    def on_open_sewage_pump_btn_clicked(self):
        """
        sewage pump stops
        """
        self.clean_status = 'pumpsewagerotate'
        self.set_status_txt("with sewage water collecting back")
        self.controlBoard.cmd_clean(self.clean_status)

    @pyqtSlot()
    def on_unit_touch_window_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        self.ur_status = "urmove"
        pub = self.ur.pub
        q = self.ur.get_final_q()
        self.ur.ur_move_to_point(pub, q)
        self.set_status_txt("going to touch the window")

    @pyqtSlot()
    def on_unit_back_init_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        self.ur_status = "urback"
        pub = self.ur.pub
        q = self.ur.get_init_q()
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

    def set_ab_height(self):
        h = self.controlBoard.rcr.get_ab_h()
        self.height_ab__rcr_lineEdit.setText(str(h))

    @pyqtSlot()
    def on_stop_rcr_btn_clicked(self):
        """
        RCR Emergency stops
        """
        self.status = 'stop'
        self.controlBoard.car_status = "STOP"
        self.set_status_txt(self.status)
        self.controlBoard.cmd_move(self.status)
        self.statusBar.showMessage("Emergency stop cars!!!", 2000)
        self.set_ab_height()

    @pyqtSlot()
    def on_down_rcr_btn_clicked(self):
        """
        RCR down step moves fixed height, fixed speed
        """
        self.status = 'down'
        height = self.height_rcr_lineEdit.text()
        vel = self.speed_rcr_lineEdit.text()
        #        self.speed_rcr_hSlider.setValue(vel)
        #        self.height_rcr_hSlider.setValue(height)
        height = -int(height)
        vel = int(vel)
        direction = "DOWN"
        self.controlBoard.cmd_precise_move(height, vel)
        self.set_rcr_txt(height, vel, direction)
        self.set_status_txt(self.status)
        self.set_ab_height()

    @pyqtSlot()
    def on_up_rcr_btn_clicked(self):
        """
        RCR up step moves, fixed height, fixed speed
        """
        self.status = 'up'
        height = self.height_rcr_lineEdit.text()
        vel = self.speed_rcr_lineEdit.text()
        height = int(height)
        vel = int(vel)
        self.controlBoard.cmd_precise_move(height, vel)
        direction = "UP"
        #        txt = "given target: " + str(height) + ", "+  "vel: " + str(vel)
        self.set_rcr_txt(height, vel, direction)
        self.set_status_txt(self.status)
        self.set_ab_height()

    @pyqtSlot()
    def on_step_up_rcr_btn_clicked(self):
        """
        when btn pressed, rcr keeps up
        """
        self.status = 'up'
        self.set_status_txt(self.status)
        self.controlBoard.cmd_move(self.status)
        self.set_ab_height()

    @pyqtSlot()
    def on_step_down_rcr_btn_clicked(self):
        """
        when btn released, rcr stops
        """
        self.status = 'down'
        self.set_status_txt(self.status)
        self.controlBoard.cmd_move(self.status)
        self.set_ab_height()

    @pyqtSlot()
    def on_cam_base_adjust_btn_clicked(self):
        """
        adjust cam servo motors
        """
        pitch = self.cam_base_pitch_hSlider.value()
        yaw = self.cam_base_yaw_hSlider.value()
        pitch, yaw = self.controlBoard.cmd_cam_adjust(pitch, yaw)
        status = "goint to angles as, pitch: " + str(pitch) + ", yaw: " + str(yaw)
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
        pitch, yaw = self.controlBoard.get_basecam_angles()
        self.cam_base_pitch_hSlider.setValue(pitch)
        self.cam_base_yaw_hSlider.setValue(yaw)
        self.controlBoard.go_init_basecam_angle()
        status = "goint to INIT angles as, pitch: " + str(pitch) + ", yaw: " + str(yaw)
        self.cam_set_status_txt(status)

    @pyqtSlot()
    def on_cam_base_set_angle_btn_clicked(self):
        """
        set the init base cam angle
        """
        pitch = self.cam_base_pitch_hSlider.value()
        yaw = self.cam_base_yaw_hSlider.value()
        pitch, yaw = self.controlBoard.set_init_basecam_angle(pitch, yaw)
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
        self.ur_info_textEdit.setText(txt)

    @pyqtSlot()
    def on_ur_launch_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        # self.ur.roslaunch()
        self.set_ur_info_txt("launch UR...")
        self.ur.Init_node()

    @pyqtSlot()
    def on_ur_core_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        # self.ur.roscore()
        self.ur.Init_node()
        self.set_ur_info_txt("open ros.")

    @pyqtSlot()
    def on_get_init_ur_pos_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        q_list = self.ur.set_init_q()
        stri = "set INITT ur pos:" + '[' + ','.join(map(str,q_list)) + '].'
        self.set_ur_info_txt(stri)

    @pyqtSlot()
    def on_get_final_ur_pos_btn_clicked(self):
        """
        Slot documentation goes here.
        """
        q_list = self.ur.set_final_q()
        print(q_list)

        stri = "set FINAL ur pos:" + '[' + ','.join(map(str,q_list)) + '].'
        self.set_ur_info_txt(stri)
