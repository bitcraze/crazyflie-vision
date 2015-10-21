import sys
import logging
import zmq
import simplejson
import time
import math

from threading import Thread

from threading import Timer, Lock

from PyQt4 import QtGui, uic
from PyQt4.QtCore import pyqtSignal, Qt, pyqtSlot, QDir, QUrl
from PyQt4.QtGui import QLabel, QActionGroup, QMessageBox, QAction, QDesktopServices

# 172.16.13.38


(main_window_class,
main_windows_base_class) = (uic.loadUiType('ksp.ui'))

sp_cmd = {
    "version": 1,
    "set-points": {
        "roll": 0,
        "pitch": 0,
        "yaw": 0,
        "velocity": 0
    }
}

class MainUI(QtGui.QMainWindow, main_window_class):
    def __init__(self, *args):
        super(MainUI, self).__init__(*args)
        self.setupUi(self)
        context = zmq.Context()
        control_conn = context.socket(zmq.PUSH)
        control_conn.bind("tcp://*:5124")

        self.pos_roll = 0
        self.pos_pitch = 0
        self.pos_rotation = 0
        self._last_heading = ""
        self.turn_lag = 0

        self.auto_pos_1.clicked.connect(lambda: self._set_rp_autopos("NW"))
        self.auto_pos_2.clicked.connect(lambda: self._set_rp_autopos("N"))
        self.auto_pos_3.clicked.connect(lambda: self._set_rp_autopos("NE"))
        self.auto_pos_4.clicked.connect(lambda: self._set_rp_autopos("W"))
        self.auto_pos_5.clicked.connect(lambda: self._set_rp_autopos(""))
        self.auto_pos_6.clicked.connect(lambda: self._set_rp_autopos("E"))
        self.auto_pos_7.clicked.connect(lambda: self._set_rp_autopos("SW"))
        self.auto_pos_8.clicked.connect(lambda: self._set_rp_autopos("S"))
        self.auto_pos_9.clicked.connect(lambda: self._set_rp_autopos("SE"))

        self.height_manual_up.pressed.connect(lambda: self.height_manual_vel.setValue(0.1))
        self.height_manual_up.released.connect(lambda: self.height_manual_vel.setValue(0.0))
        self.height_manual_down.pressed.connect(lambda: self.height_manual_vel.setValue(-0.1))
        self.height_manual_down.released.connect(lambda: self.height_manual_vel.setValue(0.0))

        self.yaw_manual_angle_0.clicked.connect(lambda: self.yaw_manual_angle.setValue(0))
        self.yaw_manual_angle_90.clicked.connect(lambda: self.yaw_manual_angle.setValue(90))
        self.yaw_manual_angle_180.clicked.connect(lambda: self.yaw_manual_angle.setValue(180))
        self.yaw_manual_angle_270.clicked.connect(lambda: self.yaw_manual_angle.setValue(270))

        self._ctrl_thread = _SpGenerator(self, control_conn)
        self._ctrl_thread.setDaemon(True)
        self._ctrl_thread.start()

    def _test(self):
        print "Clicked"

    def _set_rp_autopos(self, pos):
        old_pitch = self.pos_pitch
        old_roll = self.pos_roll
        self.pos_pitch = 0
        self.pos_roll = 0
        if "S" in pos:
            self.pos_pitch -= self.rp_auto_pos_offset.value()
        if "N" in pos:
            self.pos_pitch += self.rp_auto_pos_offset.value()
        if "E" in pos:
            self.pos_roll += self.rp_auto_pos_offset.value()
        if "W" in pos:
            self.pos_roll -= self.rp_auto_pos_offset.value()
        angle = math.atan2(self.pos_roll - old_roll, self.pos_pitch - old_pitch)
        print math.degrees(angle)
        self.pos_rotation = math.degrees(angle)
        self.turn_lag = 150 # (abs(self.pos_rotation) / 200.0) / 0.01
        print "Lag is {} and angle {}".format(self.turn_lag, self.pos_rotation)

        self._last_heading = pos

    def closeEvent(self, event):
        self.hide()


class _SpGenerator(Thread):
    """Handles incoming packets and sends the data to the correct receivers"""
    def __init__(self, ui, connection):
        Thread.__init__(self)
        self._connection = connection
        self._ui = ui
        self._set_daemon()

    def run(self):
        yaw = 0
        roll = 0
        pitch = 0
        velocity = 0
        rp_angle = 0
        delay = 0.01
        while(True):
            if self._ui.yaw_auto.isChecked():
                if self._ui.yaw_auto_position.isChecked():
                    yaw = self._ui.pos_rotation
                if self._ui.yaw_auto_rotation.isChecked():
                    yaw += self._ui.yaw_auto_rotation_speed.value() * delay
                    yaw = yaw % 360
            else:
                yaw = self._ui.yaw_manual_angle.value()

            if self._ui.rp_auto.isChecked():
                if self._ui.rp_auto_circle.isChecked():
                    roll = math.sin(math.radians(rp_angle)) * self._ui.rp_auto_circle_radius.value()
                    pitch = math.cos(math.radians(rp_angle)) * self._ui.rp_auto_circle_radius.value()
                    rp_angle += self._ui.rp_auto_circle_speed.value() * delay
                if self._ui.rp_auto_pos.isChecked():
                    if self._ui.turn_lag > 0:
                        self._ui.turn_lag -= 1
                    else:
                        roll = self._ui.pos_roll
                        pitch = self._ui.pos_pitch
            else:
                roll = self._ui.rp_manual_roll.value()
                pitch = self._ui.rp_manual_pitch.value()
            sp_cmd["set-points"]["yaw"] = yaw
            sp_cmd["set-points"]["velocity"] = self._ui.height_manual_vel.value()
            sp_cmd["set-points"]["roll"] = roll
            sp_cmd["set-points"]["pitch"] = pitch
            #print sp_cmd["set-points"]
            self._connection.send_json(sp_cmd)
            #Run at 10 HZ
            time.sleep(delay)

from PyQt4.QtGui import QApplication
app = QApplication(sys.argv)
main_window = MainUI()
main_window.show()
sys.exit(app.exec_())

