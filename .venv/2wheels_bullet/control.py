import numpy as np
import math
import pybullet as p

# Mouvement automatique de l'objectif
autoMovingGoal = False
robotRadius = 0.1/2
wheelRadius = 0.05/2

r = wheelRadius
d = robotRadius

def carre(t):
    """
    Le robot suit un carré
    """
    mod = np.fmod(t, 2)
    if mod < 1:
        # On avance pendant 1s à 500px/s
        return controle(0.25, 0) #0.5
    else:
        # On tourne pendant 1s à PI/s
        return controle(0, math.pi/2) #pi/2

def controle(x, theta):
    """
    A partir de la vitesse X (en pixels/s) et de rotation (en rad/s),
    cette fonction produit les vitesses des roues correspondantes
    """
    w1 = (x+theta*d)/r
    w2 = (x-theta*d)/r

    #print("w1= %.2f , w2= %.2f" % (w1,w2))
    return [w1,w2]



def cercle(t):
    """
    Le robot tourne sur lui même
    """
    #return np.matrix([5, -5]).T
    return controle(0.25,math.pi/2)


def updateWheels(t, speed, robotPos, robotYaw, goalPos):
    """Appelé par le simulateur pour mettre à jour les vitesses du robot

    Arguments:
        t {float} -- temps écoulé depuis le début [s]
        speed {float[2]} -- les vitesses entrées dans les curseurs [m/s], [rad/s]
        robotPos {float[2]} -- position du robot dans le repère monde [m], [m]
        robotYaw {float} -- orientation du robot dans le repère monde [rad]
        goalPos {float[2]} -- position cible du robot dans le repère monde

    Returns:
        float[2] -- les vitesses des roues [rad/s]
    """
    xRobot = robotPos[0]
    yRobot = robotPos[1]

    xCible = goalPos[0]
    yCible = goalPos[1]


    ##2) Cinematique
    x = speed[0] #vitesse roues m/s
    theta = speed[1] #vitesse rotation rad/s
    #wheelSpeeds = controle(x, theta)

    ##3) Dessin de carré
    #wheelSpeeds = carre(t)

    ##4) Mode cercle
    wheelSpeeds = cercle(t)

    return wheelSpeeds
