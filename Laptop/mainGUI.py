#!/usr/bin/python3
from PyQt5 import QtWidgets, uic, QtGui
from PyQt5.QtCore import Qt

from PyQt5.QtMultimedia import QCamera, QCameraInfo
from PyQt5.QtMultimediaWidgets import QCameraViewfinder
from PyQt5.QtWidgets import QApplication, QWidget

import ui_MainWindow

from commLaptop import Client

import sys
from pyPS4Controller.controller import Controller
import threading

import serial

from OpenGLWidget import OpenGLWidget
from PyQt5.QtGui import QSurfaceFormat
from PyQt5.QtCore import QTimer

import math

import ui_RoutePlanner

class RoutePlanner(QWidget):
    def __init__(self, mainGUI):
        super().__init__()
        self.mainGUI = mainGUI
        
        self.ui = ui_RoutePlanner.Ui_Form()
        self.ui.setupUi(self)
        self.setWindowTitle('RoutePlanner')

        self.currentPathList = [(0,0),(1,1)]
        self.currentLeg = 0
        self.currentPoint = 0
        self.actual_heading = 0
        self.legprogressrem=1
        self.currentTask = 'turn'

        self.enabled=False
        self.running = True
        self.adding_points = True

        self.ui.CancelButton.clicked.connect(self.cancel)
        self.ui.StartButton.clicked.connect(self.start)
        self.ui.PauseButton.clicked.connect(self.pause)
        self.ui.RemoveButton.clicked.connect(self.del_waypoint)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_time)
        self.timer.start(1000)

        self.DEGPERSEC = 40
        self.DISPERSEC = 0.3

        self.show()

    def update_time(self):
        print("Timer")
        if self.enabled and not self.adding_points:
            if self.currentTask == 'turn':
                print("Turning")
                vector = (self.currentPathList[self.currentPoint+1][0]-self.currentPathList[self.currentPoint][0],self.currentPathList[self.currentPoint+1][1]-self.currentPathList[self.currentPoint][1])
                self.desired_heading = math.atan2(vector[1], vector[0])
                self.desired_heading = math.degrees(self.desired_heading)
                print("Desired")
                print(self.desired_heading)

                angular_difference = (self.desired_heading - self.actual_heading) % 360
                if angular_difference > 180:
                    angular_difference -= 360

                if angular_difference < 0:
                    direction = "left"
                    time = int(abs(angular_difference) / self.DEGPERSEC)*1000
                    self.mainGUI.client.send_message("move left 100")
                else:
                    direction = "right"
                    time = int(angular_difference / self.DEGPERSEC)*1000
                    self.mainGUI.client.send_message("move right 100")

                self.currentTask = 'afterturn'
            elif self.currentTask == 'afterturn':
                self.mainGUI.client.send_message("stop")
                self.actual_heading = self.desired_heading
                self.ui.PlanCanvas.update()
                time = 1000
                self.currentPoint+=1
                self.currentTask='move'
            elif self.currentTask=='move':
                print("Moving")
                legdis = math.sqrt((self.currentPathList[self.currentLeg+1][0]-self.currentPathList[self.currentLeg][0])**2 + (self.currentPathList[self.currentLeg+1][1]-self.currentPathList[self.currentLeg][1])**2)
                print(legdis)
                print(self.legprogressrem)
                if self.legprogressrem * legdis > self.DISPERSEC:
                    print("Notend")
                    time = 1000
                    self.mainGUI.client.send_message("move forward 100")
                    self.legprogressrem -= self.DISPERSEC / legdis
                    self.ui.PlanCanvas.update()
                else:
                    print("close to end")
                    #Calculate smaller time
                    time = int((self.legprogressrem* legdis*1000) / self.DISPERSEC)
                    self.mainGUI.client.send_message("move forward 100")
                    self.currentTask='aftermove'
                    self.legprogressrem = 0
                    self.ui.PlanCanvas.update()
            elif self.currentTask=='aftermove':
                self.mainGUI.client.send_message("stop")

                if self.currentLeg + 2 == len(self.currentPathList):
                    msg = QtWidgets.QMessageBox()
                    msg.setIcon(QtWidgets.QMessageBox.Information)
                    msg.setText("The Path has been finished")
                    msg.setWindowTitle("Finished")
                    msg.addButton(QtWidgets.QMessageBox.Ok)

                    button = msg.exec_()
                    self.cancel()

                self.currentTask = 'turn'
                self.currentLeg +=1
                self.legprogressrem = 1
                time = 1000
        else:
            time = 1000
            self.mainGUI.client.send_message("stop")

        if self.running:
            self.timer.start(time)

    def cancel(self):
        self.close()
        self.mainGUI.ui.WheelNoneCheck.setChecked(True)
        self.enabled = False
        self.running = False
        self.timer.stop()
        self.mainGUI.client.send_message("stop")

    def start(self):
        print("Start")
        self.enabled=True

    def pause(self):
        print("Pause")
        self.enabled=False

    def add_waypoint(self, loc):
        print("Adding")
        print(loc)
        print(self.ui.PlanCanvas.height())
        print(self.ui.PlanCanvas.width())
        self.currentPathList.append(((-loc.y()+(self.ui.PlanCanvas.height()/2))/100,(loc.x()-(self.ui.PlanCanvas.width()/2))/100))
        print(self.currentPathList)
        self.ui.PlanCanvas.update()

    def del_waypoint(self):
        if len(self.currentPathList)>=2:
            self.currentPathList.pop()
            self.ui.PlanCanvas.update()

class SerialReader:
    def __init__(self, port='/dev/ttyACM0', baudrate=9600, timeout=1, callback=None):

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.callback = callback
        self.running = False
        self.thread = threading.Thread(target=self.read_from_serial)
    
    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            return False
        except:
            return True

    def start(self):
        self.running = True
        self.thread.start()
    
    def stop(self):
        self.running = False
        try:
            self.thread.join()
            self.ser.close()
        except:
            pass
    
    def read_from_serial(self):
        while self.running:
            line = self.ser.readline().decode('utf-8').rstrip()
            if line and self.callback:
                self.callback(line)



class MyController(Controller):
    
    def __init__(self, client ,checkbox, **kwargs):
        Controller.__init__(self, **kwargs)
        self.client = client
        self.checkbox = checkbox
        self.motor_strength = 0
        self.dir = 'stop'
        self.coord = [0,0]
        self.THRESHOLD = 30000

    def checkSend(self,message):
        if True:
            self.client.send_message(message)

    def on_x_press(self):
       self.checkSend("power on")

    def on_square_press(self):
        self.checkSend("power off")

    def update_speed(self):
        x, y = self.coord
        if y > self.THRESHOLD:
            self.dir = "right"
        elif y < -self.THRESHOLD:
            self.dir = "left"
        elif x > self.THRESHOLD:
            self.dir = "back"
        elif x < -self.THRESHOLD:
            self.dir = "forward"
        else:
            self.dir = "stop"
            self.checkSend("move forward 0")
            return

        print("Update Speed")
        print(self.dir)
        print(self.motor_strength)
        self.checkSend("move "+self.dir +" "+str(self.motor_strength))

    def on_R3_up(self,value):
        self.coord[0] = value
        self.update_speed()

    def on_R3_down(self,value):
        self.coord[0] = value
        self.update_speed()

    def on_R3_left(self,value):
        self.coord[1] = value
        self.update_speed()

    def on_R3_right(self,value):
        self.coord[1] = value
        self.update_speed()

    def on_L3_up(self, value):
        STRENGTH_DIVIDER = -330
        print("Strength:")
        print(round(value / STRENGTH_DIVIDER))
        self.motor_strength = round(value / STRENGTH_DIVIDER)
        self.update_speed()



class MainApp(QtWidgets.QMainWindow):
    def __init__(self):     
        super(MainApp, self).__init__()

        self.ui = ui_MainWindow.Ui_MainWindow()
        self.ui.setupUi(self)

        self.client = None
        self.connected = False

        self.controller_connected = False

        self.selected_servo = 1
        self.key_to_servo = {
            Qt.Key_1: 1,
            Qt.Key_2: 2,
            Qt.Key_3: 3,
            Qt.Key_4: 4,
            Qt.Key_5: 5,
            Qt.Key_6: 6
        }
        self.servo_values = {i: 1000 for i in range(1, 7)}


        self.power = False

        self.show_connect_dialog()

        self.armSerial = SerialReader(callback=self.virtualArm)
        while self.armSerial.connect():
            msg = QtWidgets.QMessageBox()
            msg.setIcon(QtWidgets.QMessageBox.Critical)
            msg.setText("Cannot Connect to the VArm")
            msg.setWindowTitle("Can't Connect")
            msg.addButton(QtWidgets.QMessageBox.Retry)
            msg.addButton(QtWidgets.QMessageBox.Cancel)

            button = msg.exec_()

            if button == QtWidgets.QMessageBox.Cancel:
                self.armSerial = None
                self.ui.ArmVArmCheck.setEnabled(False)
                break


        self.armDataPrev = [[0,0,0,0,0],[0,0,0,0,0]]


        self.controller = MyController(self.client, self.ui.WheelJoystickCheck,interface="/dev/input/js0", connecting_using_ds4drv=False)

        self.listen_thread = threading.Thread(target=lambda: self.controller.listen(on_connect=self.controller_connect, on_disconnect=self.controller_disconnect, timeout=5))
        # Start the thread
        self.listen_thread.start()

        msg = QtWidgets.QMessageBox()
        msg.setIcon(QtWidgets.QMessageBox.Information)
        msg.setText("Connect the controller")
        msg.setWindowTitle("Connect")
        msg.addButton(QtWidgets.QMessageBox.Ok)
        msg.addButton(QtWidgets.QMessageBox.Cancel)

        button = msg.exec_()
        if button != QtWidgets.QMessageBox.Cancel:
            while not self.controller_connected:
                msg = QtWidgets.QMessageBox()
                msg.setIcon(QtWidgets.QMessageBox.Critical)
                msg.setText("Cannot Connect to the Controller")
                msg.setWindowTitle("Can't Connect")
                msg.addButton(QtWidgets.QMessageBox.Retry)
                msg.addButton(QtWidgets.QMessageBox.Cancel)

                button = msg.exec_()

                if button == QtWidgets.QMessageBox.Cancel:
                    self.controller = None
                    self.ui.WheelJoystickCheck.setEnabled(False)
                    break
                elif button == QtWidgets.QMessageBox.Retry:
                    self.listen_thread.join()
                    self.controller = MyController(self.client, self.ui.WheelJoystickCheck,interface="/dev/input/js0", connecting_using_ds4drv=False)
                    self.listen_thread = threading.Thread(target=lambda: self.controller.listen(on_connect=self.controller_connect, on_disconnect=self.controller_disconnect, timeout=5))
                    self.listen_thread.start()
                    import time
                    time.sleep(1)
        else:
            self.controller = None
            self.ui.WheelJoystickCheck.setEnabled(False)



        self.ui.ConnectButton.clicked.connect(self.show_connect_dialog)
        self.ui.DisconnectButton.clicked.connect(self.disconnect)

        self.ui.OpenControlButton.clicked.connect(self.openControl)

        self.ui.commandSend.clicked.connect(self.send_message)

        self.ui.TogglePower.clicked.connect(self.toggle_power)

        self.ui.RefreshButton.clicked.connect(self.referesh)

        self.ui.PowerIndicator.setText("Rover is Off")

        self.ui.ArmVArmCheck.toggled.connect(self.arm_checkbox_state_changed)

        self.ui.WheelAutomaticCheck.toggled.connect(self.enable_automatic)

        self.ui.ReconnectControllerButton.clicked.connect(self.reconnect_controller)

        # fmt = QSurfaceFormat()
        # fmt.setVersion(3, 3)
        # fmt.setProfile(QSurfaceFormat.CoreProfile)
        # QSurfaceFormat.setDefaultFormat(fmt)

        # self.ui.openGLWidget = OpenGLWidget(parent = self.ui.Tab3d)
        # self.timer = QTimer()
        # self.timer.timeout.connect(self.ui.openGLWidget.update)
        # self.timer.start(16)  # approximately 60 fps
        # self.ui.openGLWidget.load_stl('canon.stl')

        available_cameras = QCameraInfo.availableCameras()
        print(available_cameras)
        if not available_cameras:
            print("No cameras found.")
            return

        self.camera = QCamera(available_cameras[0])
        self.camera.setViewfinder(self.ui.CameraView)  # Use the promoted widget
        # self.camera.setCaptureMode(QCamera.CaptureStillImage)
        self.camera.start()

        print("Done with setup")

    def enable_automatic(self):
        if self.ui.WheelAutomaticCheck.isChecked():
            self.route_planner = RoutePlanner(self)
        else:
            if self.route_planner!=None:
                self.route_planner.cancel()

    def reconnect_controller(self):
        print("Reconnecting")
        self.listen_thread.join()
        self.controller = MyController(self.client, self.ui.WheelJoystickCheck,interface="/dev/input/js0", connecting_using_ds4drv=False)
        self.listen_thread = threading.Thread(target=lambda: self.controller.listen(on_connect=self.controller_connect, on_disconnect=self.controller_disconnect,timeout=5))
        self.listen_thread.start()

        import time
        time.sleep(1)
        if not self.controller_connected:
            msg = QtWidgets.QMessageBox()
            msg.setIcon(QtWidgets.QMessageBox.Critical)
            msg.setText("Failed to connect controller, try again")
            msg.setWindowTitle("Can't connect")
            msg.addButton(QtWidgets.QMessageBox.Ok)

            msg.exec_()

    def controller_connect(self):
        print("Controller connected")
        self.controller_connected = True
    
    def controller_disconnect(self):
        print("Controller disconnected")
        self.controller_connected = False

        msg = QtWidgets.QMessageBox()
        msg.setIcon(QtWidgets.QMessageBox.Critical)
        msg.setText("Controller Disconnected Press Reconnect Button")
        msg.setWindowTitle("Disconnect")
        msg.addButton(QtWidgets.QMessageBox.Ok)

        msg.exec_()

        print("Dones")

        # while not self.controller_connected:
        #     msg = QtWidgets.QMessageBox()
        #     msg.setIcon(QtWidgets.QMessageBox.Critical)
        #     msg.setText("Controller Disconnected")
        #     msg.setWindowTitle("Disconnect")
        #     msg.addButton(QtWidgets.QMessageBox.Retry)
        #     msg.addButton(QtWidgets.QMessageBox.Cancel)

        #     button = msg.exec_()

        #     if button == QtWidgets.QMessageBox.Cancel:
        #         self.controller = None
        #         self.ui.WheelJoystickCheck.setEnabled(False)
        #         break
        #     elif button == QtWidgets.QMessageBox.Retry:
        #         # self.listen_thread.join()
        #         self.controller = MyController(self.client, self.ui.WheelJoystickCheck,interface="/dev/input/js0", connecting_using_ds4drv=False)
        #         self.listen_thread = threading.Thread(target=lambda: self.controller.listen(on_connect=self.controller_connect, on_disconnect=self.controller_disconnect,timeout=5))
        #         self.listen_thread.start()
                

    def arm_checkbox_state_changed(self, state):
        if self.ui.ArmVArmCheck.isChecked():
            self.armSerial.start()
        else:
            self.armSerial.stop()
            self.armSerial = SerialReader(callback=self.virtualArm)
            self.armSerial.connect()

    def virtualArm(self,data):
        splitData = [int(number) for number in data.split(',')]
        # print("Data")
        # print(splitData)

        self.averagedData = [0,0,0,0,0]
        for i in range(5):
            self.averagedData[i] = (splitData[i]+self.armDataPrev[1][i]+self.armDataPrev[0][i])/3
            self.averagedData[i] = splitData[i]

        self.armDataPrev[1]=self.armDataPrev[0]
        self.armDataPrev[0]=splitData

        for i in range(5):
            self.servo_values[i+1] = round(500+ (self.averagedData[i]/1023) *2000)

        print(self.servo_values)
        self.client.send_message("arm "+str(self.servo_values[1])+" 1")
        self.client.send_message("arm "+str(self.servo_values[2])+" 2")
        self.client.send_message("arm "+str(self.servo_values[3])+" 3")
        self.client.send_message("arm "+str(self.servo_values[4])+" 4")
        self.client.send_message("arm "+str(self.servo_values[5])+" 5")

    def referesh(self):
        self.ui.RoverStatusBox.setPlainText(self.client.status)

    def toggle_power(self):
        if self.power == False:
            self.power = True
            self.client.send_message("power on")
            self.ui.PowerIndicator.setText("Rover is On")
        elif self.power == True:
            self.power = False
            self.client.send_message("power off")
            self.ui.PowerIndicator.setText("Rover is Off")

    def keyReleaseEvent(self,event):
        if event.isAutoRepeat():
            return
        if event.key() in (Qt.Key_A, Qt.Key_D, Qt.Key_S, Qt.Key_W) and self.ui.WheelKeyboardCheck.isChecked():
            self.client.send_message("stop")

    def keyPressEvent(self, event):
        # Capture the key press event
        key = event.key()

        if self.ui.WheelKeyboardCheck.isChecked():
            if key == Qt.Key_W:
                self.client.send_message("move forward 50")
            elif key == Qt.Key_A:
                self.client.send_message("move left 50")
            elif key == Qt.Key_S:
                self.client.send_message("move back 50")
            elif key == Qt.Key_D:
                self.client.send_message("move right 50")

        if self.ui.ArmKeyboardCheck.isChecked():
            if key in self.key_to_servo:
                self.selected_servo = self.key_to_servo[key]
            
            if key == Qt.Key_R:
                if self.servo_values[self.selected_servo] < 2500:
                    self.servo_values[self.selected_servo] += 10
                    self.client.send_message("arm "+str(self.servo_values[self.selected_servo])+" "+str(self.selected_servo))
            if key == Qt.Key_F:
                self.client.send_message("arm 1000 "+str(self.selected_servo))
                self.servo_values[self.selected_servo] = 1000
            if key == Qt.Key_V:
                if self.servo_values[self.selected_servo] > 500:
                    self.servo_values[self.selected_servo] -= 10
                    self.client.send_message("arm "+str(self.servo_values[self.selected_servo])+" "+str(self.selected_servo))



    def send_message(self):
        if self.client!=None:
            self.client.send_message(str(self.ui.commandText.text()))

    def openControl(self):
        self.control = QtWidgets.QWidget()
        uic.loadUi("ButtonControl.ui",self.control)
        self.control.setWindowTitle('Second Window')
        self.control.show()

    def disconnect(self):
        if self.connected ==True:
            self.connected = False
            self.ui.ConnectionStatus.setText("Disconnected")
            if self.client!=None:
                self.client.close()
                

    def show_connect_dialog(self):
        if self.connected == False:
            dialog = QtWidgets.QDialog()
            uic.loadUi('connect.ui', dialog)  # Load your dialog UI

            ip = dialog.findChild(QtWidgets.QLineEdit, 'IpEdit')
            port = dialog.findChild(QtWidgets.QLineEdit, 'PortEdit')

            result = dialog.exec_() 

            if result == QtWidgets.QDialog.Accepted:
                ip = ip.text()
                port = port.text()
                print("Connected to:")
                print(ip, port)
                self.client = Client(ip,port)

                if self.client.connect() == True:
                    self.connected = True
                    self.ui.ConnectionStatus.setText("Connected")
                else:
                    msg = QtWidgets.QMessageBox()
                    msg.setIcon(QtWidgets.QMessageBox.Critical)
                    msg.setText("Cannot Connect to the Rover")
                    msg.setWindowTitle("Can't Connect")
                    msg.exec_()

                    self.show_connect_dialog()

    def closeEvent(self,event):
        if self.client!=None:
            self.client.close()

        if self.armSerial!=None:
            self.armSerial.stop()
        
        if self.camera!= None:
            self.camera.stop()

        import sys
        print("Exiting")
        sys.exit(0)

if __name__ == "__main__":
    print("Beginning GUI")
    loginApp = QtWidgets.QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(loginApp.exec_())
