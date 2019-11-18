import numpy as np
import math

# Mouvement automatique de l'objectif
autoMovingGoal = False
#rayon robot:
d=0.8
#rayon roue robot:
r=0.325


def matCineInv(r,d):

    angleW1=math.pi/2 + math.pi/3
    angleW2=math.pi/2 - math.pi/3
    angleW3=math.pi/2
    return  np.array([[cos(angleW1), sin(angleW1), -d],
                        [cos(angleW2), sin(angleW2), -d],
                        [cos(angleW3), sin(angleW3), -d]])/r

def matRotation(alpha):
    return np.array ([[math.cos(alpha), -math.sin(alpha)],
                        [math.sin(alpha), math.cos(alpha)]])


def carre(t):
    """
    Le robot suit un carré
    """
    mod = np.fmod(t, 2)
    if mod < 1:
        # On avance pendant 1s à 500px/s
        return controle(1, -1, -1)
    else:
        # On tourne pendant 1s à PI/s
        return controle((2*math.pi/3), 2*math.pi/3, 2*math.pi/3)

def carreHolo(t):

    return 1


#implémentation de la fonction controle
def controle(vx, vy, vtheta):
    """
    A partir de la vitesse X (en pixels/s) et de rotation (en rad/s),
    cette fonction produit les vitesses des roues correspondantes
    """
    wheel1 = (vx+vtheta*d)/r
    wheel2 = (vx-vtheta*d)/r
    wheel3 = (vx-vtheta*d)/r

    return [wheel1, wheel2, wheel3]


def updateWheels(t, speed, robotPos, robotYaw, goalPos):
    """Appelé par le simulateur pour mettre à jour les vitesses du robot

    Arguments:
        t {float} -- temps écoulé depuis le début [s]
        speed {float[3]} -- les vitesses entrées dans les curseurs [m/s], [m/s], [rad/s]
        robotPos {float[2]} -- position du robot dans le repère monde [m], [m]
        robotYaw {float} -- orientation du robot dans le repère monde [rad]
        goalPos {float[2]} -- position cible du robot dans le repère monde

    Returns:
        float[3] -- les vitesses des roues [rad/s]
    """
    robotPosX=robotPos[0]
    robotPosY=robotPos[1]

    goalPosX=goalPos[0]
    goalPosY=goalPos[1]

    vx=speed[0]
    vy=speed[1]
    vtheta=speed[2]

    #Kinv = np.array([[v1x, v1y,d],[v2x, v2y,d],[v3x,v3y,d]])/r


    #wheelSpeed=controle(vx,vy,vtheta)

    #wheelSpeed=carre(t)

    return wheelSpeed
