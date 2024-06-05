from PyQt5 import QtWidgets, uic, QtGui
from PyQt5.QtCore import Qt


from commLaptop import Client

import sys


class MainApp(QtWidgets.QMainWindow):
    def __init__(self):
        
        super(MainApp, self).__init__()

        self.client = None
        self.connected = False

        self.power = False
        self.moveDir = "s"

        uic.loadUi('rover_program.ui', self)
        self.show()
        self.show_connect_dialog()

        self.findChild(QtWidgets.QPushButton, 'ConnectButton').clicked.connect(self.show_connect_dialog)
        self.findChild(QtWidgets.QPushButton, 'DisconnectButton').clicked.connect(self.disconnect)

        self.findChild(QtWidgets.QPushButton, 'OpenControlButton').clicked.connect(self.openControl)

        self.ConnectionStatus = self.findChild(QtWidgets.QLabel, "ConnectionStatus")

        self.commandText =  self.findChild(QtWidgets.QLineEdit,"commandText")
        self.findChild(QtWidgets.QPushButton, 'commandSend').clicked.connect(self.send_message)

        self.powerIndicator = self.findChild(QtWidgets.QLabel, "PowerIndicator")
        self.findChild(QtWidgets.QPushButton, 'TogglePower').clicked.connect(self.toggle_power)

        self.WheelKeyboardCheck = self.findChild(QtWidgets.QCheckBox, "WheelKeyboardCheck")
        self.WheelJoystickCheck = self.findChild(QtWidgets.QCheckBox, "WheelJoystickCheck")
        self.WheelButtonsCheck = self.findChild(QtWidgets.QCheckBox, "WheelButtonsCheck")
        self.ArmKeyboardCheck = self.findChild(QtWidgets.QCheckBox, "ArmKeyboardCheck")
        self.ArmJoystickCheck = self.findChild(QtWidgets.QCheckBox, "ArmJoystickCheck")
        self.ArmButtonsCheck = self.findChild(QtWidgets.QCheckBox, "ArmButtonsCheck")

    def toggle_power(self):
        if self.power == False:
            self.power = True
            self.client.send_message("power on")
            self.powerIndicator.setText("Rover is On")
        elif self.power == True:
            self.power = False
            self.client.send_message("power off")
            self.powerIndicator.setText("Rover is Off")

    def keyPressEvent(self, event):
        # Capture the key press event
        key = event.key()

        if self.WheelKeyboardCheck.isChecked():
            if key == Qt.Key_W:
                if self.moveDir =="b":
                    self.client.send_message("stop")
                    self.moveDir = "s"
                else:
                    self.client.send_message("move forward 50")
                    self.moveDir = "f"
            elif key == Qt.Key_A:
                if self.moveDir =="r":
                    self.client.send_message("stop")
                    self.moveDir = "s"
                else:
                    self.client.send_message("move left 50")
                    self.moveDir = "l"
            elif key == Qt.Key_S:
                if self.moveDir =="f":
                    self.client.send_message("stop")
                    self.moveDir = "s"
                else:
                    self.client.send_message("move back 50")
                    self.moveDir = "b"
            elif key == Qt.Key_D:
                if self.moveDir =="l":
                    self.client.send_message("stop")
                    self.moveDir = "s"
                else:
                    self.client.send_message("move right 50")
                    self.moveDir = "r"

    def send_message(self):
        if self.client!=None:
            print("Sending")
            print(str(self.commandText.text()))
            self.client.send_message(str(self.commandText.text()))

    def openControl(self):
        print("Opening Control")
        self.control = QtWidgets.QWidget()
        
        uic.loadUi("ButtonControl.ui",self.control)
        self.control.setWindowTitle('Second Window')
        self.control.show()

    def disconnect(self):
        if self.connected ==True:
            self.connected = False
            self.ConnectionStatus.setText("Disconnected")
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
                    self.ConnectionStatus.setText("Connected")
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

if __name__ == "__main__":
    print("Beginning GUI")
    loginApp = QtWidgets.QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(loginApp.exec_())