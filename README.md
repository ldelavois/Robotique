# Robotique
* Projets de robotique dans le cadre du Master 2 ASPIC
* Langage: Python

* Les projets se trouvent dans le dossier .venv.
* Les sujets des projets se trouvent à l'adresse : https://gregwar.com/aspic/
* Le sujet du TD td_mgi_mgd est directement dans le dossier au format pdf.

* Il faut se placer dans le dossier spécifique à un TD et éxecuter la commande : python3 control.py 
* Les autres commandes pour tester mon code se trouvent dans les sujets de TD.

## Explication des TDs:

### geometry
Description:
* TD d'initiation à pyGame. Déplacement d'un robot à deux roues sur un plan 2D.
* Rotation autour de l'origine et rotation autour de la souris.
* Utilisation de la matrice de rotation.

### 2wheels
Description :
* TD sur la plateforme pyGame. Déplacement d'un robot à deux roues sur un plan 2D.
* Implémentation de la vitesse et trajectoire du robot en fonction de la vitesse injectée dans chacune des roues.
* Déplacement en formant un carré, un cercle, une spirale, un polygone etc....
* Utilisation de la cinématique directe.

### 2wheels_bullet
Description :
* TD sur la plateforme pyBullet. Déplacement d'un robot à deux roues dans un environnement 3D.
* Implémentation de la vitesse et de la trajectoire du robot en fonction de la vitesse injectée dans chacune des roues.
* Suivi d'un objectif.
* Utilisation de la cinématique inverse.

### holo
Description:
* TD sur la plateforme pyBullet. Déplacement d'un robot holonome à trois roues dans un environnement 3D.
* Implémentation d'un déplacement en carré.
* Utilisation de la cinématique inverse.

### trajectories
Description:
* TD sur la plateforme pyBullet. Contrôle de la trajectoire d'un robot holonome à trois roues dans un environnement 3D.
* Implémentation de contrôles de commande (PID), boucle ouverte, boucle fermée.
* Utilisation de la cinématique inverse, spline linéaire et spline cubique.
* Début d'implémentation du Bang Bang.

### td_mgd_mgi
Description:
* TD sur la plateforme pyBullet. 
* Atteinte d'une cible par un robot RT (1 joint rotoïde et 1 joint prismatique).
* Atteinte d'une cible par un robot RRR (3 joints rotoïdes).
* Implémentation du modèle géométrique direct et inverse
* Début d'implémentation de la jacobienne

