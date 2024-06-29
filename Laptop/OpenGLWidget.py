import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QOpenGLWidget
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QSurfaceFormat
from OpenGL.GL import *
from OpenGL.GLU import gluPerspective
import numpy as np
from stl import mesh

class OpenGLWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super(OpenGLWidget, self).__init__(parent)
        self.angle = 0
        self.model = None

    def initializeGL(self):
        glClearColor(0.0, 0.0, 0.0, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w / h, 1, 100)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -10.0)
        glRotatef(self.angle, 0.0, 1.0, 0.0)

        if self.model is not None:
            glBegin(GL_TRIANGLES)
            for facet in self.model.vectors:
                glNormal3fv(np.cross(facet[1] - facet[0], facet[2] - facet[0]))
                for vertex in facet:
                    glVertex3fv(vertex)
            glEnd()

        self.angle += 1.0
        if self.angle >= 360.0:
            self.angle -= 360.0
        self.update()

    def load_stl(self, filename):
        self.model = mesh.Mesh.from_file(filename)
        self.update()

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle('PyQt OpenGL STL Renderer')
        self.opengl_widget = OpenGLWidget(self)
        self.setCentralWidget(self.opengl_widget)
        self.timer = QTimer()
        self.timer.timeout.connect(self.opengl_widget.update)
        self.timer.start(16)  # approximately 60 fps

if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    # Set the OpenGL format for the application
    fmt = QSurfaceFormat()
    fmt.setVersion(3, 3)
    fmt.setProfile(QSurfaceFormat.CoreProfile)
    QSurfaceFormat.setDefaultFormat(fmt)
    
    window = MainWindow()
    window.opengl_widget.load_stl('path/to/your/file.stl')  # Replace with the path to your STL file
    window.show()
    sys.exit(app.exec_())
