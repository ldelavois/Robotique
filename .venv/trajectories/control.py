import json
import math
import numpy as np
from abc import ABC, abstractmethod


class BangBang:
    """
    BangBang control law on acceleration along a single dimension
    """
    def __init__(self, src, dst, startT, maxSpeed, maxAcc):
        """startT is used to specify the starting time of the trajectory"""
        self.src=src
        self.dst=dst
        self.startT=startT
        self.maxSpeed=maxSpeed
        self.maxAcc = maxAcc

        raise RuntimeError("Not implemented")

    def getTarget(self, t):

        #position = (self.dest-self.src)/()*(t-)
        if self.dst > self.src:
            vitesse = self.maxSpeed
        else :
            vitesse = -self.maxSpeed

        #return [position, vitesse]
        raise RuntimeError("Not implemented")

class PIDController:
    def __init__(self, kp, ki, kd, maxCartOrder, maxThetaOrder):
        """
        Parameters
        ----------
        kp : float
        ki : float
        kd : float
        maxCartOrder : float
            The maximal cartesian speed
        maxThetaOrder : float
            The maximal angular speed
        """

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.maxCartOrder = maxCartOrder
        self.maxThetaOrder = maxThetaOrder % (2*np.pi)
        self.sommeErreurs = 0
        self.erreurPrec = 0

    def step(self, t, pos, target, feedForward = np.array([0,0,0])):
        """
        Parameters
        ----------
        t : float
            time [s] used for integration
        pos : np.array[3]
            The current position of the robot
        target : np.array[3]
            The target of the robot
        feedForward : np.array[3]
            The initially expected order

        ordre
        Returns
        -------
        np.array[3]
           The resulting order
        """

        #initialisation de l'ordre
        order = np.array([0,0,0])

        #implémentation de l'erreur
        erreur = target-pos
        print("erreur = ",erreur)

        #PID:
        #valeur integrale
        #self.sommeErreurs+= erreur*t
        #print("sommeErreurs = ",self.sommeErreurs)

        #valeur derivée
        # variationErreur = [0,0,0]
        # if t>0:
        #     variationErreur = (erreur - self.erreurPrec)/t
        #print("variationErreur = ",variationErreur)
        if erreur.all() >= 0:
            order = self.kp * erreur
        else:
            order = self.kp * -erreur

        # for i in range(3):
        #
        #         if erreur[i] > 0:
        #             #order[i] = self.kp * (erreur[i] + self.ki * self.sommeErreurs[i] + self.kd * variationErreur[i])
        #             order[i] = self.kp * erreur[i]
        #         else:
        #             order[i] = -self.kp * erreur[i]
        #             #order[i] = -self.kp * (erreur[i] + self.ki * self.sommeErreurs[i] + self.kd * variationErreur[i])


        #on définit les seuils de saturation que les ordres ne doivent pas dépasser (borne max et borne min)
        if order[0] > self.maxCartOrder:
            order[0] = self.maxCartOrder
        elif order[0] < -self.maxCartOrder:
             order[0] = -self.maxCartOrder

        if order[1] > self.maxCartOrder:
            order[1]= self.maxCartOrder
        elif order[1] < -self.maxCartOrder:
            order[1] = -self.maxCartOrder

        #l'orientation est modulo 2*pi, les bornes sont différentes car on travaille sur un angle
        if order[2] > self.maxThetaOrder :
            order[2] = -self.maxThetaOrder
        elif order[2] < -self.maxThetaOrder :
            order[2]= self.maxThetaOrder

        #partie closeLoop
        #à implémenter
        self.erreurPrec = erreur
        return order

class Trajectory(ABC):

    @abstractmethod
    def getTarget(self, t):
        """
        Parameters
        ----------
        t : float
            Actual time [s]

        Returns
        -------
        targetPos : np.array[3]
            The target position (x[m],y[m],theta[rad])
        targetVel : np.array[3]
            The target velocity (vx[m/s], vy[m/s], vTheta [rad/s])
        """


    def getTargetPos(self, t):
        return self.getTarget(t)[0]

    def getTargetVel(self, t):
        return self.getTarget(t)[1]

class CompositeTrajectory(Trajectory):
    def __init__(self):
        self.trajectories = []


    def getTarget(self, t):
        targetPos = []
        targetVel = []
        for i in range(3):
            p, v = self.trajectories[i].getTarget(t)
            targetPos.append(p)
            targetVel.append(v)
        return [np.array(targetPos),np.array(targetVel)]


class FixedTarget(Trajectory):
    def __init__(self, order):
        """
        Parameters
        ----------

        order : np.array[3]
              x [m], y[m], theta [rad] position of the robot in world referential
        """
        self.order = order

    def getTarget(self,t):
        return self.order, np.array([0,0,0])

class LinearTarget(Trajectory):
    def __init__(self, speed):
        """
        Parameters
        ----------
        speed : np.array[3]
              vx [m/s], vy[m/s], vTheta [rad/s] wished speed in world referential
        """
        self.speed = speed

    def getTarget(self,t):
        return t * self.speed, self.speed

class BangBangPose2D(CompositeTrajectory):
    def __init__(self, src, dst, start_t, maxSpeed, maxAcc, maxThetaSpeed, maxThetaAcc):
        """
        Parameters
        ----------
        src : np.array[3]
              x [m], y[m], theta [rad] position of the robot in world referential at start
        dst : np.array[3]
              x [m], y[m], theta [rad] position of the robot in world referential at end
        """
        self.src=src
        self.dst=dst
        self.start_t=startT
        self.maxSpeed=maxSpeed
        self.maxAcc = maxAcc
        self.maxThetaSpeed = maxThetaSpeed
        self.maxThetaAcc = maxThetaAcc

    def getTarget(self, t):


        return [targetPos, targetVel]

class BangBangWheel(CompositeTrajectory):
    def __init__(self, src, dst, start_t, maxWheelSpeed, maxWheelAcc):
        raise RuntimeError("Not implemented")

class HoloSquare(Trajectory):
    def __init__(self, size, maxSpeed,maxAcc, maxThetaSpeed, maxThetaAcc):
        raise RuntimeError("Not implemented")

    def getTarget(self, t):
        raise RuntimeError("Not implemented")

class DirectSquare(Trajectory):
    def __init__(self, size, maxSpeed,maxAcc, maxThetaSpeed, maxThetaAcc):
        raise RuntimeError("Not implemented")

    def getTarget(self, t):
        raise RuntimeError("Not implemented")

class LinearSpline:
    def __init__(self, knots):
        """
        Parameters
        ----------
        knots : list((float,float))
            The list of couples (time, position)
        """
        self.knots=knots


    def getTarget(self, t):
        #renvoie couple [position,vitesse]
        i =0
        #on se place dans la bonne borne de temps en fonction de t
        while t > self.knots[i][0]:
            i+=1


        #on calcule la vitesse et la position avec la formule du cours
        if t>0 and t <self.knots[-1][0]:
            #print("intervalle [{:},{:}]".format(self.knots[i-1][0],self.knots[i][0]))
            #print("x1-x0 = ",self.knots[i][1]-self.knots[i-1][1])
            #print("t1-t0 = ",self.knots[i][0]-self.knots[i-1][0])

            position = (self.knots[i][1]-self.knots[i-1][1])/(self.knots[i][0]-self.knots[i-1][0])*(t-self.knots[i-1][0]) + self.knots[i-1][1]
            vitesse = (t - self.knots[i-1][0])* \
            ((self.knots[i][1]-self.knots[i-1][1])/(self.knots[i][0]-self.knots[i-1][0])) \
            + self.knots[i-1][1]

        else:
            vitesse = 0
            position = self.knots[i][1]

        return [position, vitesse]


class LinearSplinePose2D(CompositeTrajectory):
    def __init__(self, path):
        self.path = path
        self.trajectories = []

        with open(self.path) as json_data:
            data_dict = json.load(json_data)

        self.trajectories.append(data_dict["x"])
        self.trajectories.append(data_dict["y"])
        self.trajectories.append(data_dict["theta"])

        # print("x:\t",self.trajectories[0])
        # print("y:\t",self.trajectories[1])
        # print("theta:\t",self.trajectories[2])

        spline=LinearSpline(self.trajectories)
        # spline[1]=LinearSpline(self.trajectories[1])
        # spline[2]=LinearSpline(self.trajectories[2])
        #print("spline: ", spline )


    def getTarget(self, t):

        targetPos = []
        targetVel = []

        for i in range(3):
            p, v = LinearSpline(self.trajectories[i]).getTarget(t)
            targetPos.append(p)
            targetVel.append(v)

        return [targetPos, targetVel]
         #return [np.array(targetPos),np.array(targetVel)]



class CubicSpline:
    def __init__(self, knots):
        """
        Parameters
        ----------
        knots : list((float,float, float))
            The list of couples (time, position, derivée)
        """
        self.knots=knots

    def getTarget(self, t):
        #renvoie couple [position,vitesse]
        i=0
        #on se place dans la bonne borne de temps en fonction de t
        while t > self.knots[i][0]:
            i+=1


        #calcul des coefficients c2 et c3
        if t>0 and t<self.knots[-1][0]:
            print("intervalle [{:},{:}]".format(self.knots[i-1][0],self.knots[i][0]))
            c2 = (3*(self.knots[i][1]-self.knots[i-1][1])) / ((self.knots[i][0]-self.knots[i-1][0])**2)\
                - (2*self.knots[i-1][2] + self.knots[i][2]) / (self.knots[i][0]-self.knots[i-1][0])
            print("c2 = ",c2)

            c3 = (2*(self.knots[i-1][1] - self.knots[i][1])) / ((self.knots[i][0]-self.knots[i-1][0])**3) \
                + (self.knots[i-1][2] + self.knots[i][2]) / ((self.knots[i][0]-self.knots[i-1][0])**2)
            print("c3 = ",c3)

            xt0=self.knots[i-1][1]
            dxt0=self.knots[i-1][2]
            deltaT=t-self.knots[i-1][0]

            # print("x(t0) = ",self.knots[i-1][1])
            # print("dx(t0) = ",self.knots[i-1][2])
            # print("(t-t0) = ",(t-self.knots[i-1][0]))
            # print("dxt0*(t-t0) = ",(self.knots[i-1][2]*(t-self.knots[i-1][0])))
            # print("c2*(t-t0)² = ",(c2*((t-self.knots[i-1][0])**2)))
            # print("c3*(t-t0)³ = ",(c3*((t-self.knots[i-1][0])**3)))
            # print("position = ", (c2*(deltaT**2)+ c3*(deltaT**3)))

            #on calcule la position avec la formule du cours : http://www.math.univ-metz.fr/~croisil/M1-0809/2
            position = xt0 + dxt0*deltaT + c2*(deltaT**2)+ c3*(deltaT**3)
            #on calcule la vitesse avec la formule du cours : http://www.math.univ-metz.fr/~croisil/M1-0809/2
            vitesse= self.knots[i-1][1] + 2*c2*((t-self.knots[i-1][0])**2) + 3*c3*((t-self.knots[i-1][0])**2)
        else:
            vitesse = 0
            position = self.knots[i][1]

        #la position est la position la plus proche


        return [position, vitesse]


class CubicSplinePose2D(CompositeTrajectory):
    def __init__(self, path):
        self.path = path
        self.trajectories = []

        with open(self.path) as json_data:
            data_dict = json.load(json_data)

        self.trajectories.append(data_dict["x"])
        self.trajectories.append(data_dict["y"])
        self.trajectories.append(data_dict["theta"])

        #print(self.trajectories)
        #spline=CubicSpline(self.trajectories)


    def getTarget(self, t):

        targetPos = []
        targetVel = []

        for i in range(3):
            p, v = CubicSpline(self.trajectories[i]).getTarget(t)
            targetPos.append(p)
            targetVel.append(v)

        return [targetPos, targetVel]



########
#code implémenté pour la boucle ouverte
#######

#rayon robot:
d=0.08
#rayon roue robot:
r=0.0325

def matCineInv():

    #Définition de nos angles:
    angleW1= -5*math.pi/6
    angleW2= -math.pi/6
    angleW3= math.pi/2

    #on retourne notre matrice de cinematique inverse ecrite dans le cours (on change le signe de d)
    return  np.array([[math.cos(angleW1), math.sin(angleW1), -d],
                        [math.cos(angleW2), math.sin(angleW2), -d],
                        [math.cos(angleW3), math.sin(angleW3), -d]]) /r


def matRotation(alpha):

    #on retourne la matrice de rotation écrite en cours
    return np.array ([[math.cos(alpha), -math.sin(alpha)],
                        [math.sin(alpha), math.cos(alpha)]])



def cartVelToWheelVel(robotPos, cartVel):
    """
    Parameters
    ----------
    robotPos : np.array[3]
        pose of the robot in world referential (x[m], y[m], theta [rad])
    cartVel : np.array[3]
        target velocity [vx,vy,vTheta] in world referential
    Returns
    -------
    np.array[3]
        wheelVelocities [w1,w2,w3]
    """

    robotYaw=robotPos[2] #orientation du robot
    vx=cartVel[0] #velocité en x du robot
    vy=cartVel[1] #velocité en y du robot
    vtheta=cartVel[2] #degré de rotation

    #on calcule les nouvelles positions du robot
    #attention à transposer (ou non) la matrice de rotation
    cartVelRobot= matRotation(robotYaw).T.dot(np.array([vx,vy]))

    #en y ajoutant en 3eme parametre le degré de rotation pour former le vecteur
    vectRobot = np.array([cartVelRobot[0],cartVelRobot[1],vtheta])

    #on appelle notre matrice de cinematique inverse
    Kinv= matCineInv()

    #on obtient notre tableau de vitesse des roues en multipliant
    #la matrice de cinematique inverse par notre vecteur
    wheelVelocities = Kinv.dot(vectRobot)

    return wheelVelocities

if __name__ == "__main__":
    # # Test de la classe LinearSpline avec la position de x
    # spline=LinearSpline([[0, 0],[4, -1],[6, -1],[10, 0]])
    # for t in range(11):
    #     p,v=spline.getTarget(t)
    #     print("Target at t={:}:\t[{:},{:}]".format(t,p,v))

    #Test de la classe LinearSplinePose2D
    print("#### Spline Lineaire ####")
    spline=LinearSplinePose2D("linear_spline_example.json")
    for t in range(11):
        p,v=spline.getTarget(t)
        print("Target at t={:}:\t[{:},{:}]\n".format(t,p,v))

    #Test de la classe CubicSpline avec la position de x
    spline=CubicSpline([[0, 0, 0],[4, -1, -0.1],[6, -1, 0.1],[10, 0, 0]])
    for t in range(11):
        p,v=spline.getTarget(t)
        print("Target at t={:}:\t[{:},{:}]\n".format(t,p,v))

    #Test de la classe CubicSplinePose2D
    print("\n#### Spline Cubique ####")
    spline=CubicSplinePose2D("cubic_spline_example.json")
    for t in range(11):
        p,v=spline.getTarget(t)
        print("Target at t={:}:\t[{:},{:}]\n".format(t,p,v))

    ##Test de la classe CubicSpline avec la position de x
    # src=0
    # dest=1
    # startT=0
    # spline=BangBang(src,dest,startT)
    # for t in range(11):
    #     p,v=spline.getTarget(t)
    #     print("Target at t={:}:\t[{:},{:}]\n".format(t,p,v))
