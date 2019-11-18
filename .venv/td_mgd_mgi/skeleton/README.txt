Etat d'avancement:
MGD marche pour les 2 robots (léger décalage néanmoins pour le RRRRobot)
MGI marche pour le RT Robot (que quand x>0et y>0)
Bornage du MGI RRRRobot (pas d'implémentation de la valeur des angles)
Implémentation des méthodes computeJacobian pour les 2 robots


Difficultés rencontrées:
Comment faut il faire pour le MGI du RTRobot quand x<0 et y<0? (j'ai essayé d'ajouter 2*pi à l'angle mais c'est faux)
Je devrais faire la symétrie de l'angle par rapport au cosinus puis ajouter pi/2 mais je n'y arrive pas
Pour le MGI RRR, j'y serais arriver en 2D je pense.
J'ai voulu bien implémenter les MGD et MGI du coup, je n'ai pas passé assez de temps sur les jacobiennes.


