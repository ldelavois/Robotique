class LinearSpline:
    def __init__(self, knots):
        """
        Parameters
        ----------
        knots : list((float,float))
            The list of couples (time, position)
        """

        self.knots=knots
        self.time=knots[0]
        self.position=knots[1]


    def getTarget(self, t):
        #renvoie couple [position,vitesse]

        for i in range(len(self.knots)):
            if t == self.time[i] and t != self.time[-1]:
                vitesse= (t - self.time[i+1])*((self.position[i+1]-self.position[i])/(self.time[i+1]-t))+self.position[i]
            if t == self.time[-1]:
                vitesse = 0
            position = self.position[i]

        return [position, vitesse]
