#!/usr/bin/python
# -*- coding: utf-8 -*-
# https://github.com/SAKErobotics/libezgripper

import sys
from libezgripper import create_connection, Gripper
from PyQt4 import QtGui, QtCore

connection = create_connection(dev_name='/dev/ttyUSB0', baudrate=57600)
connection2 = create_connection(dev_name='/dev/ttyUSB1', baudrate=57600)
gripper = Gripper(connection, 'gripper1', [1])  # left
gripper2 = Gripper(connection2, 'gripper2', [1])  # right


class GripperGUI(QtGui.QMainWindow):
    def __init__(self):
        super(GripperGUI, self).__init__()
        self.initUI()

    def initUI(self):

        calibrateButton = QtGui.QPushButton("Calibrate", self)
        calibrateButton.resize(100, 30)
        #      calibrateButton.setStyleSheet("background-color:rgb(153, 153, 153)")
        calibrateButton.clicked.connect(gripper.calibrate)
        calibrateButton.clicked.connect(gripper2.calibrate)
        calibrateButton.move(50, 10)
        calibrateButton.show()

        releaseButton = QtGui.QPushButton("Release", self)
        releaseButton.resize(600, 40)
        releaseButton.clicked.connect(gripper.release)
        releaseButton.clicked.connect(gripper2.release)
        releaseButton.move(50, 50)

        hard_closeButton = QtGui.QPushButton("Hard Close 100%", self)
        hard_closeButton.resize(200, 100)
        hard_closeButton.clicked.connect(self.submit_goto_hard_close)
        hard_closeButton.move(50, 100)

        gotoButton = QtGui.QPushButton("Medium Close 50%", self)
        gotoButton.resize(200, 100)
        gotoButton.clicked.connect(self.submit_goto_medium_close)
        gotoButton.move(250, 100)

        openButton = QtGui.QPushButton("Soft Close 10%", self)
        openButton.clicked.connect(self.submit_goto_soft_close)
        openButton.resize(200, 100)
        openButton.move(450, 100)

        gotoButton = QtGui.QPushButton("0% Open", self)
        gotoButton.resize(100, 50)
        gotoButton.clicked.connect(self.submit_goto1)
        gotoButton.move(50, 210)

        gotoButton = QtGui.QPushButton("10% Open", self)
        gotoButton.resize(100, 50)
        gotoButton.clicked.connect(self.submit_goto2)
        gotoButton.move(150, 210)

        gotoButton = QtGui.QPushButton("20% Open", self)
        gotoButton.resize(100, 50)
        gotoButton.clicked.connect(self.submit_goto3)
        gotoButton.move(250, 210)

        gotoButton = QtGui.QPushButton("30% Open", self)
        gotoButton.resize(100, 50)
        gotoButton.clicked.connect(self.submit_goto4)
        gotoButton.move(350, 210)

        gotoButton = QtGui.QPushButton("40% Open", self)
        gotoButton.resize(100, 50)
        gotoButton.clicked.connect(self.submit_goto5)
        gotoButton.move(450, 210)

        gotoButton = QtGui.QPushButton("50% Open", self)
        gotoButton.resize(100, 50)
        gotoButton.clicked.connect(self.submit_goto6)
        gotoButton.move(550, 210)

        gotoButton = QtGui.QPushButton("60% Open", self)
        gotoButton.resize(100, 50)
        gotoButton.clicked.connect(self.submit_goto7)
        gotoButton.move(150, 260)

        gotoButton = QtGui.QPushButton("70% Open", self)
        gotoButton.resize(100, 50)
        gotoButton.clicked.connect(self.submit_goto8)
        gotoButton.move(250, 260)

        gotoButton = QtGui.QPushButton("80% Open", self)
        gotoButton.resize(100, 50)
        gotoButton.clicked.connect(self.submit_goto9)
        gotoButton.move(350, 260)

        gotoButton = QtGui.QPushButton("90% Open", self)
        gotoButton.resize(100, 50)
        gotoButton.clicked.connect(self.submit_goto10)
        gotoButton.move(450, 260)

        gotoButton = QtGui.QPushButton("100% Open", self)
        gotoButton.resize(100, 50)
        #      gotoButton.setStyleSheet("background-color:yellow")
        #      gotoButton.setStyleSheet("background-color: rgb(51, 102, 153)")
        #      gotoButton.setStyleSheet("border: 1px solid rgb(255, 255, 255)")
        gotoButton.clicked.connect(self.submit_goto11)
        gotoButton.move(550, 260)

        self.statusBar()

        self.setGeometry(300, 200, 700, 350)
        self.setWindowTitle("EZGripper GUI")
        self.show()

    def submit_goto_hard_close(self):

        gripper.goto_position(0, 100)
        gripper2.goto_position(0, 100)

    def submit_goto_medium_close(self):

        gripper.goto_position(0, 50)
        gripper2.goto_position(0, 50)

    def submit_goto_soft_close(self):

        gripper.goto_position(0, 10)
        gripper2.goto_position(0, 10)

    def submit_goto_open(self):

        gripper.goto_position(100, 100)
        gripper2.goto_position(100, 100)

    def submit_goto1(self):

        gripper.goto_position(1, 100)
        gripper2.goto_position(1, 100)

    def submit_goto2(self):

        gripper.goto_position(10, 100)
        gripper2.goto_position(10, 100)

    def submit_goto3(self):

        gripper.goto_position(20, 100)
        gripper2.goto_position(20, 100)

    def submit_goto4(self):

        gripper.goto_position(30, 100)
        gripper2.goto_position(30, 100)

    def submit_goto5(self):

        gripper.goto_position(40, 100)
        gripper2.goto_position(40, 100)

    def submit_goto6(self):

        gripper.goto_position(50, 100)
        gripper2.goto_position(50, 100)

    def submit_goto7(self):

        gripper.goto_position(60, 100)
        gripper2.goto_position(60, 100)

    def submit_goto8(self):

        gripper.goto_position(70, 100)
        gripper2.goto_position(70, 100)

    def submit_goto9(self):

        gripper.goto_position(80, 100)
        gripper2.goto_position(80, 100)

    def submit_goto10(self):

        gripper.goto_position(90, 100)
        gripper2.goto_position(90, 100)

    def submit_goto11(self):

        gripper.goto_position(100, 100)
        gripper2.goto_position(100, 100)

    def submit_goto12(self):

        gripper.goto_position(.20, 100)
        gripper2.goto_position(.20, 100)

    def submit_goto13(self):

        gripper.goto_position(.20, 100)
        gripper2.goto_position(.20, 100)

    def submit_goto14(self):

        gripper.goto_position(.20, 100)
        gripper2.goto_position(.20, 100)


def main():

    ezgripper_app = QtGui.QApplication(sys.argv)
    ex = GripperGUI()
    sys.exit(ezgripper_app.exec_())


if __name__ == '__main__':
    main()
