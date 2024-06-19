import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton
from PyQt5.QtMultimedia import QCamera
from PyQt5.QtMultimediaWidgets import QCameraViewfinder

class WebcamViewer(QWidget):
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("PyQt5 Webcam Viewer")
        self.setGeometry(300, 300, 800, 600)
        
        # Create a QCamera object
        self.camera = QCamera()
        
        # Create a QCameraViewfinder object
        self.viewfinder = QCameraViewfinder(self)
        self.viewfinder.show()
        
        # Create UI elements
        self.startButton = QPushButton('Start Camera', self)
        self.stopButton = QPushButton('Stop Camera', self)

        # Set layout
        layout = QVBoxLayout()
        layout.addWidget(self.viewfinder)
        layout.addWidget(self.startButton)
        layout.addWidget(self.stopButton)
        self.setLayout(layout)
        
        # Connect buttons to methods
        self.startButton.clicked.connect(self.start_camera)
        self.stopButton.clicked.connect(self.stop_camera)
        
        # Set the viewfinder for the camera
        self.camera.setViewfinder(self.viewfinder)

    def start_camera(self):
        # Start the camera
        self.camera.start()

    def stop_camera(self):
        # Stop the camera
        self.camera.stop()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    viewer = WebcamViewer()
    viewer.show()
    sys.exit(app.exec_())

