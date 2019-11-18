import numpy as np
from PyQt5.QtCore import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL import *
from PyQt5.QtOpenGL import *
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
import sys
import time
from resources import qInitResources

qInitResources()

# A simple OpenGL viewer based on Py QT and OpenGL
class QGLViewer(QWidget):
    def __init__(self, controlWidget):
        super().__init__()

        glf = QGLFormat.defaultFormat()
        glf.setSampleBuffers(True)
        glf.setSamples(4)
        QGLFormat.setDefaultFormat(glf)

        # Initializes the layout
        self.widget = glWidget(self)
        self.controlWidget = controlWidget
        mainLayout = QGridLayout()
        mainLayout.setContentsMargins(0, 0, 0, 0)
        mainLayout.setSpacing(0)
        mainLayout.addWidget(self.widget, 0, 0)
        mainLayout.addWidget(self.controlWidget, 0, 1)
        mainLayout.setColumnMinimumWidth(0, 800)
        mainLayout.setColumnMinimumWidth(1, 200)
        mainLayout.setColumnStretch(0, 1)
        mainLayout.setColumnStretch(1, 0)

        self.setLayout(mainLayout)

        # Refresh timeout to redraw the scene @50Hz
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(20)
        self.resize(800, 600)

        # Mouse
        self.pressed = False
        self.last = None

    # Drawing function
    def gl_draw(self):
        pass

    # Update the rendering on the OpenGL widget
    def update(self):
        self.widget.updateGL()

    # Handling mouse press for rotation
    def mousePressEvent(self, event):
        if event.button() == 1:
            self.pressed = True
            self.last = (event.x(), event.y())

    # Mouse release event to stop rotating
    def mouseReleaseEvent(self, event):
        if event.button() == 1:
            self.pressed = False

    # Rotating
    def mouseMoveEvent(self, event):
        if self.pressed:
            dX = event.x()-self.last[0]
            dY = event.y()-self.last[1]
            self.last = (event.x(), event.y())
            self.widget.pan += dX
            self.widget.tilt += dY

    # Zoom in/out
    def wheelEvent(self, event):
        self.widget.r -= event.angleDelta().y()*0.0001


class glWidget(QGLWidget):
    def __init__(self, parent):
        QGLWidget.__init__(self, parent)
        self.viewer = parent

        # Distance to the scene, pan and tilt
        self.r = 0.5
        self.pan = 30
        self.tilt = 30

    def paintGL(self):
        """
        Updating the OpenGL painting
        """
        # Viewport and clear color
        glViewport(0, 0, self.w, self.h)
        glClearColor(0.0, 0.0, 0.0, 1.0)

        # Usin 45° fov perspective
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, self.w/self.h, 0.1, 1000000.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # Clearing
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Camera
        glTranslatef(0, 0, -self.r)
        glRotatef(self.tilt, 1, 0, 0)
        glRotatef(self.pan, 0, 1, 0)
        glRotatef(-90, 1, 0, 0)
        glScalef(1, 1, 1)

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_BLEND)
        glEnable(GL_COLOR_MATERIAL)

        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.5, 0.5, 0.5, 0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.5, 0.5, 0.5, 1))
        glLightfv(GL_LIGHT0, GL_SPECULAR, (0, 0, 0, 1))

        self.viewer.gl_draw()

    def resizeGL(self, w, h):
        """
        Updating dimensions if window is resized
        """
        self.w = w
        self.h = h

    def initializeGL(self):
        glShadeModel(GL_SMOOTH)
        glEnable(GL_LINE_SMOOTH)
        glEnable(GL_MULTISAMPLE)


if __name__ == '__main__':
    """
    Example application, shows a cube
    """

    class Example(QGLViewer):
        """
        A simple example winsow
        """

        def __init__(self):
            super().__init__()
            self.verticies = (
                (1, -1, -1),
                (1, 1, -1),
                (-1, 1, -1),
                (-1, -1, -1),
                (1, -1, 1),
                (1, 1, 1),
                (-1, -1, 1),
                (-1, 1, 1))
            self.edges = (
                (0, 1),
                (0, 3),
                (0, 4),
                (2, 1),
                (2, 3),
                (2, 7),
                (6, 3),
                (6, 4),
                (6, 7),
                (5, 1),
                (5, 4),
                (5, 7))

        def gl_draw(self):
            glLineWidth(3)
            glBegin(GL_LINES)

            # Painting some vectrices
            for e in self.edges:
                A = self.verticies[e[0]]
                B = self.verticies[e[1]]
                glVertex3fv(A)
                glVertex3fv(B)

            glEnd()

    app = QApplication(sys.argv)
    window = Example()
    window.show()
    app.exec_()
