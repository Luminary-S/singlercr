#!/usr/bin/venv2 python
# -*- coding: utf-8 -*-

from PyQt5.QtCore import QThread, pyqtSignal, QWaitCondition, QMutex

'''cam thread '''
class SerialThread(QThread):
    # define a signal, when cam opens, emit this signal


    def __init__(self):
        super(SerialThread, self).__init__()
        #        self.tnum = num
        self.isstop = True
        self.cond = QWaitCondition()
        self.mutex = QMutex()
        # self.sensor = ControlBoard()

    def __del__(self):
        # 线程状态改变与线程终止
        self.isstop = True
        self.quit()

    def pause(self):
        #        print("thread pause")
        self.isstop = True

    def resume(self):
        self.isstop = False
        self.cond.wakeAll()

    def run(self):
        pass