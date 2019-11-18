import math
import numpy as np

# Cette fonction est appellée à 100Hz (100 fois par seconde)
def updateRobotPos(robotPos, mousePos):

    ##3) rotation autour de l'axe central
    # x = robotPos[0]
    # y = robotPos[1]
    # theta = math.atan2(y,x)
    # r = math.sqrt(pow(x,2)+pow(y,2))
    # alpha = 0.05
    # thetap = theta + alpha
    # robotPos[0] = math.cos(thetap)*r
    # robotPos[1] = math.sin(thetap)*r

    # #4) matrice de rotation
    # vect= np.matrix(robotPos).T
    # theta = 0.08        #correspond à la vitesse de rotation
    # r = np.array(( (np.cos(theta), -np.sin(theta)), (np.sin(theta),  np.cos(theta)) ))
    # vect=r.dot(vect) #multiplication matricielle
    # robotPos[0]=vect[0]
    # robotPos[1]=vect[1]

    # #5)tourner autour de la souris
    # vectMouse=np.matrix(mousePos).T
    # vectRobot=np.matrix(robotPos).T
    # theta = 0.05        #correspond à la vitesse de rotation
    # r = np.array(( (np.cos(theta), -np.sin(theta)), (np.sin(theta),  np.cos(theta)) ))
    # vectRobot= vectMouse + r.dot(vectRobot-vectMouse)
    # robotPos[0]=vectRobot[0]
    # robotPos[1]=vectRobot[1]

    # 6) avec coordonnées homogènes
    # vectRobot=(np.matrix(robotPos).T,1) #a corriger pour ajouter 1 en dimension 3
    # theta = 0.05        #correspond à la vitesse de rotation
    # r = np.matrix(( (np.cos(theta), -np.sin(theta), 0), (np.sin(theta),  np.cos(theta), 0), (0,0,1) ))
    # T=np.matrix(( (1,0,mousePos[0]), (0,1,mousePos[1]), (0,0,1)))
    # print(vectRobot)
    # M=T.dot(r).dot(-T).dot(vectRobot) #a corriger (pb de dimensions pour vectRobot)
    # vectRobot=T.dot(r).dot(-T).dot(vectRobot)
    # robotPos[0]=vectRobot[0]
    # robotPos[1]=vectRobot[1]




    return robotPos
