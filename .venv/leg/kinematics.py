import time
import abc
import math
import sys
from threading import Lock
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QThread, QMutex
from scipy.optimize import minimize, root
import numpy as np
from qtgl import QGLViewer
from control import updateMotors
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtWebChannel import QWebChannel

drawLock = Lock()

# To debug, run with:
# --remote-debugging-port=8080
class ControlWidget(QWebEngineView):
    def __init__(self, parent, robot):
        super().__init__(parent)
        self.robot = robot

        channel = QWebChannel(self.page())
        self.page().setWebChannel(channel)
        channel.registerObject("api", self)

        self.load(QUrl('qrc:/www/index.html'))
        self.setFixedWidth(300)

    @pyqtSlot(result=str)
    def debugInfo(self):
        theta = self.robot.theta
        tip = self.robot.tip()
        debug = "Theta:<br/>"
        debug += "%.2f deg<br/>" % np.rad2deg(theta[0])
        debug += "%.2f deg<br/>" % np.rad2deg(theta[1])
        debug += "%.2f deg<br/>" % np.rad2deg(theta[2])
        debug += "<br/>"
        debug += "XYZ:<br/>"
        debug += "%.2f m<br/>" % tip[0]
        debug += "%.2f m<br/>" % tip[1]
        debug += "%.2f m<br/>" % tip[2]
        debug += "<br/>"
        debug += "XYZ (cible):<br/>"
        debug += "%.2f m<br/>" % robot.xyzSlider[0]
        debug += "%.2f m<br/>" % robot.xyzSlider[1]
        debug += "%.2f m<br/>" % robot.xyzSlider[2]
        return debug
    
    @pyqtSlot(float, float, float)
    def setAngles(self, angle1, angle2, angle3):
        robot.anglesSlider = [np.deg2rad(angle1), np.deg2rad(angle2), np.deg2rad(angle3)]

    @pyqtSlot(float, float, float)
    def setXYZ(self, x, y, z):
        robot.xyzSlider = [x, y, z]

    @pyqtSlot(str)
    def setMode(self, mode):
        robot.mode = mode

class ChainItem:
    """
    An item of the kinematic chain
    """

    def __init__(self):
        self.object = None
        self.matrix = np.matrix(np.eye(4))

    def gl_draw(self):
        """
        Drawing the item using OpenGL
        """
        pass


class ChainTranslation(ChainItem):
    """
    A translation in the kinematic chain
    """

    def __init__(self, translation):
        super().__init__()
        self.translation = np.array(translation)
        self.matrix[0, 3] = translation[0]
        self.matrix[1, 3] = translation[1]
        self.matrix[2, 3] = translation[2]

    def gl_draw(self):
        drawLock.acquire()
        glPushMatrix()
        glColor3f(0.75, 0.75, 0.75)
        qobj = gluNewQuadric()
        gluQuadricNormals(qobj, GLU_SMOOTH)
        cylinderAxis = np.array([0, 0, 1])
        jointAxis = np.array(
            [self.matrix[0, 3], self.matrix[1, 3], self.matrix[2, 3]])
        length = np.linalg.norm(jointAxis)
        jointAxis = jointAxis/length

        angle = math.acos(np.dot(cylinderAxis, jointAxis))
        axis = np.cross(jointAxis, cylinderAxis)

        glRotatef(np.rad2deg(-angle), axis[0], axis[1], axis[2])
        gluCylinder(qobj, 0.005, 0.005, length, 16, 16)

        gluDeleteQuadric(qobj)
        glPopMatrix()

        # glLineWidth(5)
        # glBegin(GL_LINES)
        # glColor3f(0.75, 0.75, 0.75)
        # glVertex3f(0, 0, 0)
        # glVertex3f(self.matrix[0, 3], self.matrix[1, 3], self.matrix[2, 3])
        # glEnd()

        drawLock.release()


class ChainRotation(ChainItem):
    """
    A rotation around one of the x, y or z axis
    """

    def __init__(self, axis):
        super().__init__()
        self.axis = axis

    def gl_draw(self):
        """
        Draw the rotation axis with OpenGL
        """
        glPushMatrix()
        qobj = gluNewQuadric()
        gluQuadricNormals(qobj, GLU_SMOOTH)
        if self.axis == 'x':
            glColor3ub(188, 0, 0)
            glRotatef(90, 0, 1, 0)
        if self.axis == 'y':
            glColor3ub(111, 188, 0)
            glRotatef(90, 1, 0, 0)
        if self.axis == 'z':
            glColor3ub(0, 4, 188)
            glRotatef(90, 0, 0, 1)
        glTranslatef(0, 0, -0.025)
        gluCylinder(qobj, 0.01, 0.01, 0.05, 16, 16)
        gluDisk(qobj, 0, 0.01, 16, 16)
        glTranslatef(0, 0, 0.05)
        gluDisk(qobj, 0, 0.01, 16, 16)
        gluDeleteQuadric(qobj)
        glPopMatrix()

    def set_theta(self, theta):
        """
        Sets the value of the rotation
        """
        if self.axis == 'x':
            self.matrix = np.matrix([[1,               0,                0, 0],
                                     [0, math.cos(theta), -math.sin(theta), 0],
                                     [0, math.sin(theta),  math.cos(theta), 0],
                                     [0,                0,               0, 1]])
        if self.axis == 'y':
            self.matrix = np.matrix([[math.cos(theta), 0, math.sin(theta), 0],
                                     [0, 1,               0, 0],
                                     [-math.sin(theta), 0, math.cos(theta), 0],
                                     [0, 0,               0, 1]])
        if self.axis == 'z':
            self.matrix = np.matrix([[math.cos(theta), -math.sin(theta), 0, 0],
                                     [math.sin(theta),  math.cos(theta), 0, 0],
                                     [0,                0, 1, 0],
                                     [0,                0, 0, 1]])


class Robot:
    """
    Building the robot
    """

    def __init__(self):
        self.anglesSlider = [0, 0, 0]
        self.xyzSlider = [0, 0, 0]
        self.mode = ''
        self.chain = []
        self.joints = {}
        self.matrix = np.matrix(np.eye(4))

        self.add_translation([0.04, 0, 0])
        self.add_joint('leg_yaw', 'z')
        self.add_translation([0.045, 0, 0])
        self.add_joint('leg_roll1', 'y')
        self.add_translation([0.065, 0, 0])
        self.add_joint('leg_roll2', 'y')
        self.add_translation([0.087, 0, 0])

        self.theta = np.array([0, 0, 0])
        self.position = np.array([-0.15, 0, 0.1])
        self.matrix[0, 3] = self.position[0]
        self.matrix[1, 3] = self.position[1]
        self.matrix[2, 3] = self.position[2]
        self.history = []
        self.mutex = QMutex()

    def add_translation(self, matrix):
        """
        Append a translation to the robot
        """
        self.chain += [ChainTranslation(matrix)]

    def add_joint(self, name, axis):
        """
        Append a joint
        """
        rotation = ChainRotation(axis)
        self.chain += [rotation]
        self.joints[name] = rotation

    def get_joint(self, name):
        """
        Getting joint
        """
        return self.joints[name]

    def gl_draw(self):
        """
        Drawing the robot in OpenGL
        """
        self.mutex.lock()
        glPushMatrix()
        glMultMatrixf(self.matrix.T)

        for item in self.chain:
            item.gl_draw()
            glMultMatrixf(item.matrix.T)

        glPopMatrix()
        self.mutex.unlock()

    def tip(self):
        """
        Computing the foot tip of the robot
        """
        matrix = np.matrix(np.eye(4))
        for item in self.chain:
            matrix = matrix*item.matrix
        return [matrix[0, 3], matrix[1, 3], matrix[2, 3]]

    def set_thetas(self, theta):
        """
        Setting all the theta values for all the joint
        """
        self.theta = theta
        robot.get_joint('leg_yaw').set_theta(theta[0])
        robot.get_joint('leg_roll1').set_theta(theta[1])
        robot.get_joint('leg_roll2').set_theta(theta[2])


class RobotWindow(QGLViewer):
    """
    Using QGLViewer for drawing
    """

    def __init__(self, robot):
        control = ControlWidget(None, robot)
        super().__init__(control)
        self.robot = robot

    def gl_draw(self):
        # Grid
        glDisable(GL_LIGHTING)
        glPushMatrix()
        glLineWidth(1)
        glColor3f(0.3, 0.3, 0.3)
        glBegin(GL_LINES)
        r = 0.5
        D = np.linspace(-r, r, 10)
        for x in D:
            glVertex3f(x, -r, 0)
            glVertex3f(x, r, 0)
        for y in D:
            glVertex3f(-r, y, 0)
            glVertex3f(r, y, 0)

        glEnd()
        glPopMatrix()

        glPushMatrix()
        glMultMatrixf(self.robot.matrix.T)
        # History
        glLineWidth(1)
        glColor3f(0.7, 0.3, 0.3)
        glBegin(GL_LINES)
        last = None
        for h in robot.history:
            if last is not None:
                glVertex3fv(last)
                glVertex3fv(h)
            last = h
        glEnd()
        glPopMatrix()
        glEnable(GL_LIGHTING)

        self.robot.gl_draw()


class AnimationThread(QThread):
    def __init__(self, robot):
        super().__init__()
        self.robot = robot
        self.t = 0

    def run(self):
        while True:
            self.t += 0.02
            time.sleep(0.02)

            # Using some functions to produce kinematics order
            drawLock.acquire()
            thetas = updateMotors(self.t, self.robot)
            drawLock.release()

            # Solving the IK
            self.robot.set_thetas(thetas)
            pos = self.robot.tip()

            # Putting the position in the history
            robot.history += [pos]
            while len(robot.history) > 100:
                robot.history.pop(0)


# Building the robot
robot = Robot()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    thread = AnimationThread(robot)
    thread.start()
    window = RobotWindow(robot)
    window.show()
    app.exec_()
