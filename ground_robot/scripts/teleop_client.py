#!/usr/bin/python3

import sys
import math
import rclpy

from functools import partial
from my_robot_interfaces.srv import CmdVehicle

from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QLineEdit


class Example(QWidget):

    def __init__(self):
        super().__init__()

        #ROS functionality
        rclpy.init()
        self.node = rclpy.create_node("teleop_key_client_node")
        self.node.get_logger().info("teleop is up!")

        #GUI funtionality
        self.initUI(self.node)



    def initUI(self,node):
        self.setGeometry(400, 400, 370, 260)
        self.setWindowTitle('Land robot Joystick')

        button_width = 50
        button_height = 50
        

        '''
            ___________x
            |
            |
            |
            |
            |y

            QRect(x, y, width, height)

        '''

        # Robot movement buttons
        self.b_forward = QPushButton(self)
        self.b_backwards = QPushButton(self)
        self.b_rotateClockwise = QPushButton(self)
        self.b_rotateAntiClockwise = QPushButton(self)
        self.b_stop = QPushButton(self)
        
        self.b_forward.setGeometry(QtCore.QRect(60, 70, button_width, button_height))
        self.b_backwards.setGeometry(QtCore.QRect(60, 170, button_width, button_height))
        self.b_rotateClockwise.setGeometry(QtCore.QRect(110, 120, button_width, button_height))
        self.b_rotateAntiClockwise.setGeometry(QtCore.QRect(10, 120, button_width, button_height))
        self.b_stop.setGeometry(QtCore.QRect(60, 120, button_width, button_height))

        self.b_forward.setText("^")
        self.b_backwards.setText("v")
        self.b_rotateClockwise.setText("->")
        self.b_rotateAntiClockwise.setText("<-")
        self.b_stop.setText("O")

        self.b_forward.clicked.connect(self.goForward)
        self.b_backwards.clicked.connect(self.goBackwards)
        self.b_rotateClockwise.clicked.connect(self.rotateClockwise)
        self.b_rotateAntiClockwise.clicked.connect(self.rotateAntiClockwise)
        self.b_stop.clicked.connect(self.stop)


        # Camera movement buttons
        self.b_camera_rotate = QPushButton(self)
        self.le_input_cam_grades = QLineEdit(self)
        self.b_camera_rotate.setText("o")
        self.le_input_cam_grades.setText("0")

        self.b_camera_rotate.setGeometry(QtCore.QRect(210, 120, button_width, button_height))
        self.le_input_cam_grades.setGeometry(QtCore.QRect(260, 120, 2*button_width, button_height))

        self.b_camera_rotate.clicked.connect(self.camRotate)


        # Labels
        self.l_robot_movements = QLabel(self)
        self.l_cam_movements = QLabel(self)

        self.l_robot_movements.setText("Robot")
        self.l_cam_movements.setText("Camera")

        self.l_robot_movements.setGeometry(QtCore.QRect(65, 5, button_width, 80))
        self.l_cam_movements.setGeometry(QtCore.QRect(255, 5, button_width, 80))

        
        self.show()


    
    # Movement functions
    # It was first planned to use the keyboard as a joystick (like a teleop_key)
    def goForward(self):
        self.send_request_to_server(self.node,"w")

    def goBackwards(self):
        self.send_request_to_server(self.node,"s")

    def rotateClockwise(self):
        self.send_request_to_server(self.node,"a")

    def rotateAntiClockwise(self):
        self.send_request_to_server(self.node,"d")

    def stop(self):
        self.send_request_to_server(self.node,"space_bar")

    def camRotate(self):
        cam_grades = self.le_input_cam_grades.text()
        if self.validate_cam_grades(cam_grades):
            cam_rads = self.normalize_cam_grades(float(cam_grades))
            self.send_request_to_server(self.node,"-> "+str(cam_rads))


    # Validations functions of cam_grades
    def validate_cam_grades(self,cam_grades):
        try:
            cam_grades = float(cam_grades)
            return True
        except Exception as e:
            return False

    def normalize_cam_grades(self,cam_grades):
        if (cam_grades >=-180.0) and (cam_grades <=180.0):
            cam_rads = math.radians(cam_grades)
        elif cam_grades>180.0:
            res = cam_grades // 180.0
            cam_grades = cam_grades - res*180.0
            cam_rads = math.radians(cam_grades)
        elif cam_grades<-180.0:
            res = -cam_grades // 180.0
            cam_grades = cam_grades + res*180.0
            cam_rads = math.radians(cam_grades)
        
        return cam_rads



    def send_request_to_server(self,node,key_pressed):

        '''
            string command
            ---
            bool result
        '''

        cmd_client_ = node.create_client(CmdVehicle,"cmd_vehicle")
        
        while not cmd_client_.wait_for_service(1.0):
            node.get_logger().warn("Waiting for Server teleop_key...")

        self.request_ = CmdVehicle.Request()
        self.request_.command = key_pressed

        future = cmd_client_.call_async(self.request_)

        future.add_done_callback(partial(self.callback_call_CmdVehicle,cmd=key_pressed))



    def callback_call_CmdVehicle(self,future,cmd):
        try:
            response = future.result()
            self.get_logger().info("Response to cmd: " + cmd + " is: " + str(response.result))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):

    app = QApplication(sys.argv)
    ex = Example()

    try:
        sys.exit(app.exec_())
    finally:
        rclpy.shutdown()
        


if __name__ == '__main__':
    main()