import math
import numpy as np

wheelRadius = 9
robotWidth = 27


def avance(t):
    """
    Le robot avance tout droit
    """
    return np.matrix([10, 10]).T


def tourne(t):
    """
    Le robot tourne sur lui même
    """
    return np.matrix([5, -5]).T


def avanceEtTourne(t):
    """
    Le robot tourne en avançant
    """
    return np.matrix([5, 25]).T


def avanceTourne(t):
    """
    Le robot avance, puis tourne, puis avance, puis tourne etc.
    """
    mod = np.fmod(t, 1)
    if mod < 0.5:
        return avance(t)
    else:
        return tourne(t)


def avanceEtTourneEtAvance(t):
    mod = np.fmod(t, 1)
    if mod < 0.5:
        return avance(t)
    else:
        return avanceEtTourne(t)


def avanceTourneRecul(t):
    """
    Le robot est supposé avancer, puis tourner dans un sens, puis
    tourner dans l'autre sens et reculer
    Il reste normalement statique
    """

    # La cinématique directe peut être obtenue en inversant les équations de la cinématique inverse, on obtient alors:

    mod = np.fmod(t, 2)
    if mod < 1:
        return avanceTourne(mod)
    else:
        return -avanceTourne(2-mod)


def spirale(t):
    """
    Le robot tourne en décrivant une spirale
    """
    return np.matrix([t, t + 10]).T


def controle(vitesseX, vitesseRot):
    """
    A partir de la vitesse X (en pixels/s) et de rotation (en rad/s),
    cette fonction produit les vitesses des roues correspondantes
    """
    wheelSpeeds= [0,0]

    wheelSpeeds[0] = (vitesseX + vitesseRot*robotWidth)/wheelRadius

    wheelSpeeds[1] = (vitesseX - vitesseRot*robotWidth)/wheelRadius


    return wheelSpeeds


def carre(t):
    """
    Le robot suit un carré
    """
    mod = np.fmod(t, 2)
    if mod < 1:
        # On avance pendant 1s à 500px/s
        return controle(300, 0)
    else:
        # On tourne pendant 1s à PI/s
        return controle(0, math.pi/2)


def updateRobotPos(robotPos, mousePos, t, dt):
    # Obtention des vitesses des roues désirées [rad/s]
    wheelSpeeds = avanceTourne(t)

    vitesseX = (wheelRadius*wheelSpeeds[0] + wheelRadius*wheelSpeeds[1])/2
    theta = (wheelRadius*wheelSpeeds[0] - wheelRadius*wheelSpeeds[1])/(2*robotWidth)
    #x' = (r*w1 + r*w2)/2
    #theta = (rw1 - rw2)/(2*d)

    print("x'=%f, theta=%f" % (vitesseX, theta))
    # A vous de jouer !

    # robotPos comporte désormais une troisième case qui correspond à
    # l'orientation du robot
    # Si on incrémente cette orientation à chaque tick, le robot va pivoter
    # sur lui même !

    robotPos[0] += vitesseX * dt * math.cos(robotPos[2])
    robotPos[1] += vitesseX * dt * math.sin(robotPos[2])
    robotPos[2] += theta * dt

    return robotPos
