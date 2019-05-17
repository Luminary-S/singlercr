#!./venv/python
# import sympy as sy
# from ur_move import UR
# from sensor_frame import Sensor
# from rcr import RCR
import rospy, time
# import copy
from singlercr.msg import rcr, sensorArduino
from SerialThread import SerialThread
# from std_msgs.msg import UInt16,Float32
import numpy as np

class RCRNode():
    def __init__(self, sensor,rcr):
        self.sensor = sensor
        self.rcr = rcr

    def init_node(self):
        # rospy.init_node("sensor_handler")
        self.pub1 = rospy.Publisher("rcr", rcr, queue_size=10)
        self.pub2 = rospy.Publisher("sensorAr", sensorArduino, queue_size=10)
        # self.sub = rospy.Subscriber("/joint_states", JointState, self.callback)

    def set_atype_np(self, type, var):
        pass

    def publish(self):
        ratet = 30
        rcrmsg = rcr()
        sensormsg = sensorArduino()

        rate = rospy.Rate(ratet)
        while not rospy.is_shutdown():
            sensormsg.force = np.uint16(self.sensor.force1)
            sensormsg.sonic = np.uint16(self.sensor.sonic1)
            rcrmsg.velocity = np.float32(self.rcr.get_vel())
            rcrmsg.ab_position = np.float32(self.rcr.get_height())
            self.pub1.publish(rcrmsg)
            self.pub2.publish(sensormsg)
            rate.sleep()

'''cam thread '''
class rcrnodeThread(SerialThread):
    # define a signal, when cam opens, emit this signal
    # controlSignal = pyqtSignal()

    def __init__(self, sensor, rcr):
        super(rcrnodeThread, self).__init__()
        self.device = RCRNode(sensor, rcr)
        self.device.init_node()

    def run(self):
        # while 1:
        self.mutex.lock()
        # print("isstop:",self.isstop)
        if self.isstop:
            self.cond.wait(self.mutex)
        self.device.publish()
        # self.controlSignal.emit()
        self.mutex.unlock()
# def main():


if __name__ == '__main__':
    rcrnode = RCRNode()
    rcrnode.init_node()
    rcrnode.publish()