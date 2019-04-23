#!/usr/bin/venv python
# -*- coding: utf-8 -*-
#from PyQt5.QtCore import pyqtSlot

from PyQt5.QtWidgets import QApplication
from RCRwindow import MainWindow
import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

if __name__ == '__main__':

    app = QApplication(sys.argv)
    ui = MainWindow()
    ui.show()
    sys.exit(app.exec_())
