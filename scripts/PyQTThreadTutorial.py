#!/usr/bin/env python
import constants
import datetime as dt
import time
import os
import multiprocessing
import ControlArdu
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtCore import pyqtSignal


font_but = QtGui.QFont()
font_but.setFamily("Segoe UI Symbol")
font_but.setPointSize(10)
font_but.setWeight(95)

class PushBut1(QtWidgets.QPushButton):

    def __init__(self, parent=None):
        super(PushBut1, self).__init__(parent)
        self.setMouseTracking(True)
        self.setStyleSheet("margin: 1px; padding: 7px; \
                           background-  color: rgba(1,255,255,100); \
                           color: rgba(0,190,255,255); \
                           border-style: solid; \
                           border-radius: 3px; border-width: 0.5px; \
                           border-color: rgba(127,127,255,255);")
    def enterEvent(self, event):
        if self.isEnabled() is True:
            self.setStyleSheet("margin: 1px; padding: 7px; \
                               background-color: rgba(1,255,255,100); \
                               color: rgba(0,230,255,255); \
                               border-style: solid; \
                               border-radius: 3px; \
                               border-width: 0.5px; \
                               border-color: rgba(0,230,255,255);")
        if self.isEnabled() is False:
            self.setStyleSheet("margin: 1px; padding: 7px; \
                               background-color: rgba(1,255,255,100); \
                               color: rgba(0,190,255,255); \
                               border-style: solid; \
                               border-radius: 3px; \
                               border-width: 0.5px; \
                               border-color: rgba(127,127,255,255);")
    def leaveEvent(self, event):
        self.setStyleSheet("margin: 1px; padding: 7px; \
                           background-color: rgba(1,255,255,100); \
                           color: rgba(0,190,255,255); \
                           border-style: solid; \
                           border-radius: 3px; \
                           border-width: 0.5px; \
                           border-color: rgba(127,127,255,255);")

class QthreadApp(QtWidgets.QWidget):
    sig = pyqtSignal(str)    
    def __init__(self, connectPipe,resolution, parent=None):
        self.connectPipe = connectPipe
        QtWidgets.QWidget.__init__(self, parent)
        self.setWindowTitle("QThread Application")
        self.setWindowIcon(QtGui.QIcon("Path/to/image/file.png"))
        self.setMinimumWidth(resolution.width() / 3)
        self.setMinimumHeight(resolution.height() / 1.5)
        self.setStyleSheet("QWidget {background-color: rgba(0,41,59,255);} \
                           QScrollBar:horizontal {width: 1px; \
                           height: 1px; \
                           background-color: rgba(0,41,59,255);} \
                           QScrollBar:vertical {width: 1px; \
                           height: 1px; \
                           background-color: rgba(0,41,59,255);}")
        self.linef = QtWidgets.QLineEdit(self)
        self.linef.setPlaceholderText("Connect to...")
        self.linef.setStyleSheet("margin: 1px; \
                                 padding: 7px; \
                                 background-color: rgba(0,255,255,100); \
                                 color: rgba(0,190,255,255); \
                                 border-style: solid; \
                                 border-radius: 3px; \
                                 border-width: 0.5px; \
                                 border-color: rgba(0,140,255,255);")
        self.textf = QtWidgets.QTextEdit(self)
        self.textf.setPlaceholderText("Results...")
        self.textf.setStyleSheet("margin: 1px; padding: 7px; \
                                 background-color: rgba(0,255,255,100); \
                                 color: rgba(0,190,255,255); \
                                 border-style: solid; \
                                 border-radius: 3px; \
                                 border-width: 0.5px; \
                                 border-color: rgba(0,140,255,255);")
        self.but1 = PushBut1(self)
        self.but1.setText("Start")
        self.but1.setFixedWidth(72)
        self.but1.setFont(font_but)
        self.but2 = PushBut1(self)
        self.but2.setText("Stop")
        self.but2.setFixedWidth(72)
        self.but2.setFont(font_but)
        self.grid1 = QtWidgets.QGridLayout()
        self.grid1.addWidget(self.linef, 0, 0, 1, 12)
        self.grid1.addWidget(self.but1, 0, 12, 1, 1)
        self.grid1.addWidget(self.but2, 0, 13, 1, 1)
        self.grid1.addWidget(self.textf, 1, 0, 13, 14)
        self.grid1.setContentsMargins(7, 7, 7, 7)
        self.setLayout(self.grid1)
        self.but1.clicked.connect(self.on_but1)
        self.but2.clicked.connect(self.on_but2)

    def on_but1(self):
     self.textf.clear()
     lineftxt = self.linef.text()
     self.thread1 = QThread1(self.connectPipe)
     self.sig.connect(self.thread1.on_source)
     self.sig.emit(lineftxt)
     self.thread1.start()
     self.thread1.sig1.connect(self.on_info)
     self.but1.setEnabled(False)
    def on_info(self, info):
        self.textf.append(str(info))
    def on_but2(self):
        try:
            self.thread1.running = False
            time.sleep(2)
            self.but1.setEnabled(True)
        except:
            pass

class QThread1(QtCore.QThread):
    sig1 = pyqtSignal(str)

    def __init__(self,connectPipe, parent=None):
        QtCore.QThread.__init__(self, parent)
        self.connect = connectPipe
    def on_source(self, lineftxt):
        self.source_txt = lineftxt
    def run(self):
        self.running = True
        while self.running:
            # talker()
            while self.connect.poll() == True:
                self.sig1.emit(self.connect.recv())
            time.sleep(1)

def qtGUIProcess(conn):
    import sys
    app = QtWidgets.QApplication(sys.argv)
    desktop = QtWidgets.QApplication.desktop()
    resolution = desktop.availableGeometry()
    myapp = QthreadApp(conn,resolution)
    myapp.setWindowOpacity(0.95)
    myapp.show()
    myapp.move(resolution.center() - myapp.rect().center())
    sys.exit(app.exec_())

if __name__ == "__main__":
    
    #create pipe
    parent_conn, child_conn = multiprocessing.Pipe()
    
    p1 = multiprocessing.Process(target=qtGUIProcess, args=(parent_conn,)) 
    p2 = multiprocessing.Process(target=ControlArdu.talker, args=(child_conn,)) 
  
    # running processes 
    p1.start() 
    p2.start() 

    # wait until processes finish 
    p1.join() 
    p2.join() 
    
else:
    desktop = QtWidgets.QApplication.desktop()
    resolution = desktop.availableGeometry()


