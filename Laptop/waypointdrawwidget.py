import sys
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QPainter, QBrush, QPen, QColor
from PyQt5.QtCore import Qt, QPoint
import math

class WayPointDrawWidget(QWidget):
    def __init__(self,routePlanner):
        super().__init__()
        self.initUI()
        self.routePlanner = routePlanner
        self.translation_x = 0
        self.translation_y = 0
        self.dragging = False

    def initUI(self):
        self.setGeometry(100, 100, 800, 600)
        self.setWindowTitle('Black Background and Blue Triangle')
        self.show()

    def paintEvent(self, event):
        painter = QPainter(self)

        # Fill the background with black
        painter.fillRect(self.rect(), QBrush(Qt.black))

        if not self.routePlanner.adding_points:
            painter.translate(0, 0)
        else:
            painter.translate(self.translation_x, self.translation_y)

        # Draw the path lines
        painter.setPen(QPen(Qt.white, 5, Qt.SolidLine))

        currentPathList=[]
        theta = math.radians(-self.routePlanner.actual_heading- 90)

        for i in range(len(self.routePlanner.currentPathList)):
            currentPathList.append([self.routePlanner.currentPathList[i][0],self.routePlanner.currentPathList[i][1]])

            if not self.routePlanner.adding_points:
                currentPathList[i][0] -= self.routePlanner.currentPathList[self.routePlanner.currentLeg][0]
                currentPathList[i][1] -= self.routePlanner.currentPathList[self.routePlanner.currentLeg][1]

                vector = (self.routePlanner.currentPathList[self.routePlanner.currentLeg+1][0]-self.routePlanner.currentPathList[self.routePlanner.currentLeg][0],self.routePlanner.currentPathList[self.routePlanner.currentLeg+1][1]-self.routePlanner.currentPathList[self.routePlanner.currentLeg][1])

                currentPathList[i][0] -= vector[0] * (1- self.routePlanner.legprogressrem)
                currentPathList[i][1] -= vector[1] * (1- self.routePlanner.legprogressrem)       

            cos_theta = math.cos(theta) #* self.width() /self.height()
            sin_theta = math.sin(theta) #* self.height() / self.width()

            tmp = currentPathList[i][0] * cos_theta - currentPathList[i][1] * sin_theta
            currentPathList[i][1] = currentPathList[i][0] * sin_theta + currentPathList[i][1] * cos_theta
            currentPathList[i][0] = tmp

            currentPathList[i][0] *= self.routePlanner.ui.ZoomSlider.value()
            currentPathList[i][1] *=self.routePlanner.ui.ZoomSlider.value()

            currentPathList[i][0] +=self.width() // 2
            currentPathList[i][1] +=self.height() // 2


        # Draw lines from self.currentPathList
        for i in range(len(currentPathList) - 1):
            start = QPoint(int(currentPathList[i][0]), int(currentPathList[i][1]))
            end = QPoint(int(currentPathList[i + 1][0]), int(currentPathList[i + 1][1]))
            painter.drawLine(start, end)

        #Start and end points
        start_point = currentPathList[0]
        painter.setPen(QPen(Qt.green, 2, Qt.SolidLine))
        painter.setBrush(QBrush(Qt.green, Qt.SolidPattern))
        painter.drawEllipse(QPoint(int(start_point[0]), int(start_point[1])), 10, 10)

        end_point = currentPathList[-1]
        painter.setPen(QPen(Qt.red, 2, Qt.SolidLine))
        painter.setBrush(QBrush(Qt.red, Qt.SolidPattern))
        painter.drawEllipse(QPoint(int(end_point[0]), int(end_point[1])), 10, 10)

        if not self.routePlanner.adding_points:
            # Set the pen and brush for the triangle
            painter.setPen(QPen(Qt.blue, 1, Qt.SolidLine))
            painter.setBrush(QBrush(Qt.blue, Qt.SolidPattern))

            # Define the points of the triangle
            center_x, center_y = self.width() // 2, self.height() // 2
            triangle_size = 10
            points = [
                QPoint(int(center_x), int(center_y - triangle_size)),
                QPoint(int(center_x - triangle_size), int(center_y + triangle_size)),
                QPoint(int(center_x + triangle_size), int(center_y + triangle_size))
            ]

            # Draw the triangle
            painter.drawPolygon(*points)

    def mousePressEvent(self, event):
        # Get the position of the click
        if event.button() == Qt.LeftButton:
            self.clicked_point = event.pos()
            print(f"Clicked at: {self.clicked_point.x()}, {self.clicked_point.y()}")
            newPos = [event.pos().x()- self.translation_x, event.pos().y()- self.translation_y]

            self.routePlanner.add_waypoint(newPos)

        if event.button() == Qt.RightButton and self.routePlanner.adding_points:
            self.last_mouse_position = event.pos()
            self.dragging = True

    def mouseMoveEvent(self, event):
        if self.dragging:
            # Calculate the distance moved
            delta = event.pos() - self.last_mouse_position
            # Update the translation values
            self.translation_x += delta.x()
            self.translation_y += delta.y()
            # Update the last mouse position
            self.last_mouse_position = event.pos()
            # Repaint the widget
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.RightButton:
            self.dragging = False


