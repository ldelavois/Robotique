import math
import numpy as np
from scipy.optimize import minimize

def updateMotors(t, robot):
    if robot.mode == 'xyz-iterative':
        target = np.array(robot.xyzSlider)
        robot.set_thetas([0, 0, 0])
        tip = robot.tip()
        print('XYZ target: '+str(robot.xyzSlider))
        print('Tip: '+str(tip))
        return [0, 0, 0]
        # TODO!
    elif robot.mode == 'xyz':
        return [0, 0, 0]
        # TODO!
    elif robot.mode == 'auto':
        return [math.sin(t), math.sin(t*1.1), math.sin(t*1.2)]
        # TODO!
    else:
        return robot.anglesSlider
