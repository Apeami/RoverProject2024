from PyQt5 import QtWidgets, uic, QtGui
from PyQt5.QtCore import Qt

from PyQt5.QtMultimedia import QCamera, QCameraInfo
from PyQt5.QtMultimediaWidgets import QCameraViewfinder

import ui_MainWindow

from commLaptop import Client

import sys
from pyPS4Controller.controller import Controller
import threading


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



        self.selected_servo = 1
        self.key_to_servo = {
            Qt.Key_1: 1,
            Qt.Key_2: 2,
            Qt.Key_3: 3,
            Qt.Key_4: 4,
            Qt.Key_5: 5,
            Qt.Key_6: 6
        }
        self.servo_values = {i: 0 for i in range(1, 7)}


        self.power = False

        self.show_connect_dialog()

    
        self.controller = MyController(self.client, self.ui.WheelJoystickCheck,interface="/dev/input/js0", connecting_using_ds4drv=False)

        listen_thread = threading.Thread(target=self.controller.listen)
        # Start the thread
        listen_thread.start()

        self.ui.ConnectButton.clicked.connect(self.show_connect_dialog)
        self.ui.DisconnectButton.clicked.connect(self.disconnect)

        self.ui.OpenControlButton.clicked.connect(self.openControl)

        self.ui.commandSend.clicked.connect(self.send_message)

        self.ui.TogglePower.clicked.connect(self.toggle_power)

        self.ui.RefreshButton.clicked.connect(self.referesh)

        self.ui.PowerIndicator.setText("Rover is Off")

        available_cameras = QCameraInfo.availableCameras()
        print(available_cameras)
        if not available_cameras:
            print("No cameras found.")
            return

        self.camera = QCamera(available_cameras[0])
        self.camera.setViewfinder(self.ui.CameraView)  # Use the promoted widget
        # self.camera.setCaptureMode(QCamera.CaptureStillImage)
        self.camera.start()

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
        print("released")
        if event.key() in (Qt.Key_A, Qt.Key_D, Qt.Key_S, Qt.Key_W):
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
                if self.servo_values[self.selected_servo] < 180:
                    self.servo_values[self.selected_servo] += 1
                    self.client.send_message("arm "+str(self.servo_values[self.selected_servo])+" "+str(self.selected_servo))
            if key == Qt.Key_F:
                self.client.send_message("arm 0 "+str(self.selected_servo))
                self.servo_values[self.selected_servo] = 0
            if key == Qt.Key_V:
                if self.servo_values[self.selected_servo] > 0:
                    self.servo_values[self.selected_servo] -= 1
                    self.client.send_message("arm "+str(self.servo_values[self.selected_servo])+" "+str(self.selected_servo))


    def send_message(self):
        if self.client!=None:
            print("Sending")
            print(str(self.ui.commandText.text()))
            self.client.send_message(str(self.ui.commandText.text()))

    def openControl(self):
        print("Opening Control")
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

        self.camera.stop()

if __name__ == "__main__":
    print("Beginning GUI")
    loginApp = QtWidgets.QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(loginApp.exec_())
