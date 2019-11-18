import json
import math
import numpy as np
from abc import ABC, abstractmethod
from scipy.optimize import minimize


vX=np.array([1,0,0])
vY=np.array([0,1,0])
vZ=np.array([0,0,1])

def matRotX(alpha):
    matrice = np.array([
                [1,     0,              0,              0],
                [0,     np.cos(alpha),  np.sin(alpha),  0],
                [0,     -np.sin(alpha), np.cos(alpha),  0],
                [0,     0,              0,               1]])
    return matrice

def dMatRotX(alpha):
    matrice = np.array([
                [0,     0,              0,              0],
                [0,     -np.sin(alpha),  np.cos(alpha),  0],
                [0,     -np.cos(alpha), -np.sin(alpha),  0],
                [0,     0,              0,               0]])
    return matrice

def matRotY(alpha):
    matrice = np.array([
                [np.cos(alpha), 0,      -np.sin(alpha), 0],
                [0,             1,      0,              0],
                [np.sin(alpha), 0,      np.cos(alpha),  0],
                [0,     0,              0,              1]])
    return matrice

def dMatRotY(alpha):
    matrice = np.array([
                [-np.sin(alpha), 0,      -np.cos(alpha), 0],
                [0,             0,      0,              0],
                [np.cos(alpha), 0,      -np.sin(alpha),  0],
                [0,     0,              0,              0]])
    return matrice

def matRotZ(alpha):
    matrice = np.array([
                [np.cos(alpha),     np.sin(alpha),   0,  0],
                [-np.sin(alpha),    np.cos(alpha),   0,  0],
                [0,                 0,               1,  0],
                [0,                 0,               0,  1]])
    return matrice

def dMatRotZ(alpha):
    matrice = np.array([
                [-np.sin(alpha),    np.cos(alpha),   0,  0],
                [-np.cos(alpha),    -np.sin(alpha),   0,  0],
                [0,                 0,               0,  0],
                [0,                 0,               0,  0]])
    return matrice

def TranslationSelonAxe(v,d):
    translation = np.identity(4)
    translation[:3,3] = v.dot(-d)
    # translation = np.array(
    #                     [1 ,    0,  0, -d*v[0]],
    #                     [0, 1,  0,      -d*v[1]],
    #                     [0,0,1, -d*v[2]],
    #                     [0,0,0, 1])
    return translation


class RobotModel:
    @abstractmethod
    def computeMGD(self, joint):
        """
        Parameters
        ----------
        joints_position : np.array
            The values of the joints of the robot in joint space

        Returns
        -------
        np.array
            The coordinate of the effectors in the operational space
            Coordonnées de la croix noire (axe [x,y] car plan)
        """

    @abstractmethod
    def computeAnalyticalMGI(self, operational_target):
        """
        Parameters
        ----------
        operational_target : np.array
            The target given in the operational space



        Returns
        -------
        np.array or None
            The target for the robot in joints space
        int
            The number of solutions
        """

    @abstractmethod
    def computeJacobian(self, joints):
        """
        Parameters
        ----------
        joints : np.array
            Current position of all the joints

        Returns
        -------
        np.array
            The jacobian of the robot
        """

class RTRobot(RobotModel):
    """
    Model a robot with a 2 degrees of freedom: 1 rotation and 1 translation

    The operational space of the robot is 2 dimensional because it can only move inside a plane
    """
    def __init__(self):
        pass

    def computeMGD(self, joints):
        self.joints = joints
        q1=self.joints[0]
        q2=self.joints[1]
        d=0.2+q2  #longueur rectangle bleu

        alpha = self.joints[0] % (2*np.pi)
        #print ("alpha = ",alpha)

        matrice0TE= np.array(matRotZ(-alpha)).dot(TranslationSelonAxe(vX,-(0.2+q2))).dot(TranslationSelonAxe(vY,0.25))

        x = matrice0TE[0][3]
        y = matrice0TE[1][3]

        #retourne les coordonnées (x,y) de la croix noire en fonction de Q
        #x = axe rouge, y = axe vert
        return x,y

    def computeAnalyticalMGI(self, operational_target):

                self.operational_target = operational_target
                xTarget = self.operational_target[0]
                yTarget = self.operational_target[1]

                L1Min = 0.2
                L1Max = 0.45
                L2 = 0.25
                D = math.sqrt(xTarget**2 + yTarget**2)
                print("D = ",D)
                L1= math.sqrt(D**2 - L2**2)
                print("L1 = ",L1)
                # rMin = math.sqrt(L1Min**2 + L2**2)
                # rMax = math.sqrt(L1Max**2 + L2**2)
                betha = math.cos(-np.pi/4)

                if L1 < L1Min or L1 > L1Max :
                    print("La target n'est pas atteignable... Bien tenté!")
                    return None, 0

                else :
                    knots = [0,0]
                    phi = np.arccos(xTarget / (math.sqrt(xTarget**2 + yTarget**2)))
                    print("phi = ",phi)
                    alpha = np.arccos((L1**2 + D**2 - L2**2)/(2*L1*D))
                    print("alpha = ",alpha)
                    if yTarget > 0 :
                        knots[0]=phi+alpha
                    if yTarget <0 and xTarget <0:
                        knots[0]= phi + alpha + np.pi/2 #innexact
                    if yTarget <0 and xTarget >0:
                        knots[0]= phi + alpha - np.pi/2 #innexact

                    knots[1]=L1-L1Min

                    print(knots)

                    return knots,1

    def computeJacobian(self, joints):

        self.joints = joints
        q1=self.joints[0]
        q2=self.joints[1]

        alpha = self.joints[0] % (2*np.pi)
        #print ("alpha = ",alpha)

        matriceJ= np.array(dMatRotZ(-alpha)).dot(TranslationSelonAxe(vX,-(0.2+q2))).dot(TranslationSelonAxe(vY,0.25))
        #print("matrice0TE = \n",matrice0TE)

        return matriceJ

class RRRRobot(RobotModel):
    """
    Model a robot with 3 degrees of freedom along different axis
    """
    def __init__(self):
        pass

    def computeMGD(self, joints):
        self.joints = joints
        q1 = self.joints[0]
        q2 = self.joints[1]
        q3 = self.joints[2]

        #Longueurs entre 2 articulations
        L0=1.00
        L1=0.41
        L2=0.29
        L3=0.31


        alpha1 = q1 % (2*np.pi)
        alpha2 = q2 % (2*np.pi)
        alpha3 = q3 % (2*np.pi)
        #print ("alpha = ",alpha)


        #Matrices successives
        matrice0T1= np.array(TranslationSelonAxe(vZ,-L0))
        matrice1T2= np.array(matRotZ(-alpha1).dot(TranslationSelonAxe(vY,-L1)).dot(TranslationSelonAxe(vZ,-0.01)))
        matrice2T3= np.array(matRotX(-alpha2).dot(TranslationSelonAxe(vY,-L2)).dot(TranslationSelonAxe(vX,-0.02)))
        matrice3T4= np.array(matRotX(-alpha3).dot(TranslationSelonAxe(vY,-L3)).dot(TranslationSelonAxe(vX,0.02)))

        #La matrice de translation 0TE est la multiplication matricielle des matrices successives
        matrice0TE = matrice0T1.dot(matrice1T2).dot(matrice2T3).dot(matrice3T4)
        #print("matrice0TE = \n",matrice0TE)

        #On récupère les coordonnées de la croix noire
        x = matrice0TE[0][3]
        y = matrice0TE[1][3]
        z = matrice0TE[2][3]
        #print(x,y,z)
        #retourne les coordonnées (x,y,z) de la croix noire en fonction de Q
        #x = axe rouge, y = axe vert
        return x,y,z
        #return [0,0,1]

    def computeAnalyticalMGI(self, operational_target):
        self.operational_target = operational_target
        xTarget = self.operational_target[0]
        yTarget = self.operational_target[1]
        zTarget = self.operational_target[1]

        #Longueurs entre 2 articulations
        L0=1.00
        L1=0.41
        L2=0.29
        L3=0.31

        #distance D : entre l'origine [0,0,1] et la target
        D = (xTarget**2 + yTarget**2 + (zTarget-L0)**2)**(1/2)
        print("D = ",D)

        if D>(L1+L2+L3) or D<(L1-L2-L3):
            print("Inatteignable")
            return None,0
        else :
            #à faire
            knots=[0,0,0]
            return knots, 1




    # D = math.sqrt(xTarget**2 + yTarget**2)
    # print("D = ",D)
    # L1= math.sqrt(D**2 - L2**2)
    # print("L1 = ",L1)
    # # rMin = math.sqrt(L1Min**2 + L2**2)
    # # rMax = math.sqrt(L1Max**2 + L2**2)
    # betha = math.cos(-np.pi/4)
    #
    # if L1 < L1Min or L1 > L1Max :
    #     print("La target n'est pas atteignable... Bien tenté!")
    #     return None, 0
    #
    # else :
    #     knots = [0,0]
    #     phi = np.arccos(xTarget / (math.sqrt(xTarget**2 + yTarget**2)))
    #     print("phi = ",phi)
    #     alpha = np.arccos((L1**2 + D**2 - L2**2)/(2*L1*D))
    #     print("alpha = ",alpha)
    #     if yTarget > 0 :
    #         knots[0]=phi+alpha
    #     if yTarget <0 and xTarget <0:
    #         knots[0]= phi + alpha + np.pi/2
    #     if yTarget <0 and xTarget >0:
    #         knots[0]= phi + alpha
    #
    #     knots[1]=L1-L1Min
    #
    #     print(knots)
    #
    #     return knots,1
        #raise RuntimeError("Not implemented")
        return knots,0

    def computeJacobian(self, joints):
        self.joints = joints
        q1=self.joints[0]
        q2=self.joints[1]
        q3=self.joints[2]

        #Longueurs entre 2 articulations
        L0=1.00
        L1=0.41
        L2=0.29
        L3=0.31


        alpha1 = q1 % (2*np.pi)
        alpha2 = q2 % (2*np.pi)
        alpha3 = q3 % (2*np.pi)
        #print ("alpha = ",alpha)


        #Matrices successives
        matrice0T1= np.array(TranslationSelonAxe(vZ,-L0))
        matrice1T2= np.array(dMatRotZ(-alpha1).dot(TranslationSelonAxe(vY,-L1)).dot(TranslationSelonAxe(vZ,-0.01)))
        matrice2T3= np.array(dMatRotX(-alpha2).dot(TranslationSelonAxe(vY,-L2)).dot(TranslationSelonAxe(vX,-0.02)))
        matrice3T4= np.array(dMatRotX(-alpha3).dot(TranslationSelonAxe(vY,-L3)).dot(TranslationSelonAxe(vX,0.02)))

        #La matrice de translation 0TE est la multiplication matricielle des matrices successives
        matriceJ = matrice0T1.dot(matrice1T2).dot(matrice2T3).dot(matrice3T4)
        #print("matrice0TE = \n",matrice0TE)

        #retourne les coordonnées (x,y,z) de la croix noire en fonction de Q
        #x = axe rouge, y = axe vert
        return matriceJ


def searchJacInv(model, joints, target):
    """
    Parameters
    ----------
    model : RobotModel
        The model of the robot used to compute MGD and Jacobian
    joints : np.array
        Initial position of the joints
    target : np.array
        The target in operational space

    Returns
    -------
    np.array
        The wished position for joints in order to reach the target
    """
    self.joints = joints
    self.target = target
    self.model = model

    if self.model == rt :
        wishJoints = np.array[0,0]
        return 0
    else :
        wishJoints = np.array[0,0,0]
        return 0

    raise RuntimeError("Not implemented")

def searchJacTransposed(model, joints, target):
    """
    Parameters
    ----------
    model : RobotModel
        The model of the robot used to compute MGD and Jacobian
    joints : np.array
        Initial position of the joints
    target : np.array
        The target in operational space

    Returns
    -------
    np.array
        The wished position for joints in order to reach the target
    """
    self.joints = joints
    self.target = target
    self.model = model

    if self.model == rt :
        wishJoints = np.array[0,0]
        return 0
    else :
        wishJoints = np.array[0,0,0]
        return 0

    raise RuntimeError("Not implemented")

if __name__ == "__main__":
    #RTRobot:
    #MGD:
    # knots = [0,0]
    # x,y = RTRobot().computeMGD(knots)
    # print("x =",x)
    # print("y =",y)

    # #MGI:
    # posTarget = [0.45,0.25]
    # knots,nbSolution = RTRobot().computeAnalyticalMGI(posTarget)
    # print("knots: ",knots)
    # print("nb solution = ",nbSolution)

    #computeJacobian
    # knots = [0,0]
    # matriceJ = RTRobot().computeJacobian(knots)
    # print("matriceJ =\n", matriceJ)



    #RRRRobot
    #MGD
    # knots = [0,0,0]
    # x,y,z = RRRRobot().computeMGD(knots)
    # print("x = ",x)
    # print("y = ",y)
    # print("z = ",z)

    #MGI
    posTarget = [1,1,1]
    knots = RRRRobot().computeAnalyticalMGI(posTarget)
    print("knots: ",knots)
    print("nb solution = ",nbSolution)

    #computeJacobian
    # knots = [0,0,0]
    # matriceJ = RRRRobot().computeJacobian(knots)
    # print("matriceJ =\n", matriceJ)
