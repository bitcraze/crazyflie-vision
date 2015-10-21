import sys
import logging
import zmq
import simplejson
import time

from plotwidget import PlotWidget
from threading import Thread

from threading import Timer, Lock

from PyQt4 import QtGui, uic
from PyQt4.QtCore import pyqtSignal, Qt, pyqtSlot, QDir, QUrl
from PyQt4.QtGui import QLabel, QActionGroup, QMessageBox, QAction, QDesktopServices

(main_window_class,
main_windows_base_class) = (uic.loadUiType('main.ui'))

class PidGraph(object):
    def __init__(self):
        return

    def update(self, data):
        return

#cmd = {
#    "version": 1,
#    "client_name": "N/A",
#    "ctrl": {
#        "roll": 0.1,
#        "pitch": 0.1,
#        "yaw": 0.0,
#        "thrust": 0.0
#    }
#}

class MainUI(QtGui.QMainWindow, main_window_class):

    #connectionLostSignal = pyqtSignal(str, str)
    #connectionInitiatedSignal = pyqtSignal(str)
    #batteryUpdatedSignal = pyqtSignal(int, object, object)
    #connectionDoneSignal = pyqtSignal(str)
    #connectionFailedSignal = pyqtSignal(str, str)
    #disconnectedSignal = pyqtSignal(str)
    #linkQualitySignal = pyqtSignal(int)

    #_input_device_error_signal = pyqtSignal(str)
    _new_data_signal = pyqtSignal(object)
    #_log_error_signal = pyqtSignal(object, str)

    def __init__(self, *args):
        super(MainUI, self).__init__(*args)
        self.setupUi(self)
        context = zmq.Context()
        self.pid_conn = context.socket(zmq.PULL)
        self.pid_conn.bind("tcp://*:5123")

        self._new_data_signal.connect(self._new_data)

        #self._graphs = {"roll": PlotWidget(fps=30), "pitch": PlotWidget(fps=30),
        #                "yaw": PlotWidget(fps=30), "thrust": PlotWidget(fps=30)}
        #self._graphs = {"position": PlotWidget(fps=30), "velocity": PlotWidget(fps=30)}
        #self._graphs = {"velocity": PlotWidget(fps=30)}
        self._graphs = {"roll": PlotWidget(fps=30), "pitch": PlotWidget(fps=30)}
        #self._graphs = {"roll": PlotWidget(fps=30)}
        #self._graphs = {"yaw": PlotWidget(fps=30)}

        for graph in self._graphs:
            self._graphs[graph].set_title(graph)
            self._graphs[graph].add_curve("P", pen='k')
            self._graphs[graph].add_curve("I", pen='m')
            self._graphs[graph].add_curve("D", pen='c')
            self._graphs[graph].add_curve("E", pen='r')
            self._graphs[graph].add_curve("SP", pen='b')
            self._graphs[graph].add_curve("OUT", pen='g')
            self.plot_layout.addWidget(self._graphs[graph])

        self._rx_thread = _IncomingDataHandler(self._new_data_signal.emit, self.pid_conn)
        self._rx_thread.setDaemon(True)
        self._rx_thread.start()

    def _new_data(self, data):
        try:
            self._graphs[data["name"]].add_data(data["data"], time.time())
            #print "{}: {}".format(data["name"], data["data"])
        except KeyError as e:
            #print e
            pass


    def closeEvent(self, event):
        self.hide()


class _IncomingDataHandler(Thread):
    """Handles incoming packets and sends the data to the correct receivers"""
    def __init__(self, cb, connection):
        Thread.__init__(self)
        self._cb = cb
        self._connection = connection
        self._set_daemon()

    def run(self):
        while(True):
            data = self._connection.recv_json()
            self._cb(data)


from PyQt4.QtGui import QApplication
app = QApplication(sys.argv)
main_window = MainUI()
main_window.show()
sys.exit(app.exec_())

