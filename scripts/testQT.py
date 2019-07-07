#!/usr/bin/env python
from PyQt5.QtWidgets import QApplication, QLabel

def start():
    app = QApplication([])
    label = QLabel('Hello World')
    label.show()
    app.exec_()

if __name__ == '__main__':
    start()