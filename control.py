import math
import numpy as np
import meshcat.transformations as tf
l1, l2, l3 = 0.045, 0.065, 0.087
l = 0.04
def floorBtw(min, max, value):
    if value < min:
        return min
    if value >=max:
        return max
    return value

def radToDeg(rad):
    return rad * 180 / math.pi

def degToRad(deg):
    return deg * math.pi / 180

def alKashi(a, b, c):
    """
    Renvoie l'angle en radians du triangle rectangle dont les côtés opposés aux angles sont de longueur a, b et c
    """
    result = floorBtw(-1,1,((a*a + b*b - c*c) / (2*a*b)))
    return math.acos(result)
def sandbox(t):
    """
    python simulator.py -m sandbox

    Un premier bac à sable pour faire des expériences

    La fonction reçoit le temps écoulé depuis le début (t) et retourne une position cible
    pour les angles des 12 moteurs

    - Entrée: t, le temps (secondes écoulées depuis le début)
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """

    # Par exemple, on envoie un mouvement sinusoidal
    targets = [0]*12
    targets[0] = np.sin(t)

    return targets

def direct(alpha, beta, gamma):
    """
    python simulator.py -m direct

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Reçoit en argument la cible (alpha, beta, gamma) des degrés de liberté de la patte, et produit
    la position (x, y, z) atteinte par le bout de la patte

    - Sliders: les angles des trois moteurs (alpha, beta, gamma)
    - Entrées: alpha, beta, gamma, la cible (radians) des moteurs
    - Sortie: un tableau contenant la position atteinte par le bout de la patte (en mètres)
    """

    xp = l1 + math.cos(beta)*l2 + math.cos(beta + gamma)*l3
    yp = math.sin(beta)*l2 + math.sin(beta + gamma)*l3
    
    x = math.cos(alpha) * xp 
    y = math.sin(alpha) * xp 
    z = yp
    
    return x, y, z

def inverse(x, y, z):
    """
    python simulator.py -m inverse

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Reçoit en argument une position cible (x, y, z) pour le bout de la patte, et produit les angles
    (alpha, beta, gamma) pour que la patte atteigne cet objectif

    - Sliders: la position cible x, y, z du bout de la patte
    - Entrée: x, y, z, une position cible dans le repère de la patte (mètres), provenant du slider
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """
    d = math.sqrt(x**2 + y**2)
    ac = math.sqrt((d - l1)**2 + z**2)

    theta1 = -math.atan2(y, x)

    gamma = math.atan2(z, d - l1) 
    beta = alKashi(ac, l2, l3)
    theta2 = beta + gamma

    theta3 = math.pi - alKashi(l2, l3, ac) - math.pi/2

    return [theta1, theta2, theta3]

def interpolation_lineaire_3d(x,y, t):
    """
    Interpolation linéaire en trois dimensions entre deux points (x0, y0, z0) et (x1, y1, z1) selon un paramètre t.

    Arguments :
    - x0, y0, z0 : coordonnées du premier point
    - x1, y1, z1 : coordonnées du deuxième point
    - t : paramètre d'interpolation (compris entre 0 et 1)

    Sortie :
    - Les coordonnées interpolées (x, y, z) entre les deux points en fonction de t
    """
    x0,y0,z0 = x
    x1,y1,z1 = y

    xr = x0 + (x1 - x0) * t
    yr = y0 + (y1 - y0) * t
    zr = z0 + (z1 - z0) * t


    return xr, yr, zr


from interpolation import Interpolate
def draw(t):
    """
    python simulator.py -m draw

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Le but est, à partir du temps donné, de suivre une trajectoire de triangle. Pour ce faire, on
    utilisera une interpolation linéaire entre trois points, et la fonction inverse précédente.

    - Entrée: t, le temps (secondes écoulées depuis le début)
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """

    duration = 10

    point1 = (0.1, -0.07, -0.07)
    point2 = (0.1, 0, 0.07)
    point3 = (0.1, 0.07,-0.07)
    
    t = t % duration
    t = t / duration

    interpolate = Interpolate([point1,point2,point3,point1])
    interpolated_point = interpolate.lerp(t)

    return inverse(interpolated_point[0], interpolated_point[1], interpolated_point[2])
from math import pi

def getLegsCoord(x,y,z,i):
    
  
    legsAlpha= [ degToRad(135),degToRad(-135), degToRad(-45), degToRad(45)]
    legsOrigin =[
        [math.cos(legsAlpha[0])*l, math.sin(legsAlpha[0])*l, 0],
        [math.cos(legsAlpha[1])*l, math.sin(legsAlpha[1])*l, 0],
        [math.cos(legsAlpha[2])*l, math.sin(legsAlpha[2])*l, 0],
        [math.cos(legsAlpha[3])*l, math.sin(legsAlpha[3])*l, 0]

    ]   
    
    translate = tf.translation_matrix(legsOrigin[i]) \
        @ tf.rotation_matrix(legsAlpha[i], [0, 0, 1]) \
    
    inversed = tf.inverse_matrix(translate)
 
 
    point = np.array([
        [x],
        [y],
        [z],
        [1.]])
    output = np.dot(inversed, point)

    return output[0][0], output[1][0], output[2][0]


def legs(targets_robot):
    """
    python simulator.py -m legs

    Le robot est figé en l'air, on contrôle toute les pattes

    - Sliders: les 12 coordonnées (x, y, z) du bout des 4 pattes
    - Entrée: des positions cibles (tuples (x, y, z)) pour le bout des 4 pattes
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """
    targets = [0]*12

    for i in range(4):
        #convert to leg coordinates
        x, y, z = getLegsCoord(targets_robot[i][0], targets_robot[i][1], targets_robot[i][2], i)
        #inverse kinematics
        angles = inverse(x, y, z)
        #add to targets
        targets[i*3] = angles[0]
        targets[i*3 + 1] = angles[1]
        targets[i*3 + 2] = angles[2]


    return targets



initial_gap= 0.10

def produit_vector(x,y):
    return (x[0]*y[0], x[1]*y[1],  x[2]*y[2])

def walk(t, speed_x, speed_y, speed_rotation):
    """
    python simulator.py -m walk

    Le but est d'intégrer tout ce que nous avons vu ici pour faire marcher le robot

    - Sliders: speed_x, speed_y, speed_rotation, la vitesse cible du robot
    - Entrée: t, le temps (secondes écoulées depuis le début)
            speed_x, speed_y, et speed_rotation, vitesses cibles contrôlées par les sliders
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """
    initial_height = 0.05

    init_1 = [-initial_gap, initial_gap, -initial_height]
    init_2 = [-initial_gap, -initial_gap, -initial_height]
    init_3 = [initial_gap, -initial_gap, -initial_height]
    init_4 = [initial_gap, initial_gap, -initial_height]


    

    duration = 1
    shift = 0.8

    
    far_point = 0.1
    near_point = far_point/4

    

    tri_point_1 = [
        init_1,
        (0, initial_gap, 0),
        init_1,
        produit_vector(init_1, (1, 1, 0)),
        init_1,
        init_1,
        init_1
    ]

    tri_point_2 = [
        init_2,
        init_2,
        init_2,
        init_2,
        (0, -initial_gap, 0),
        init_2,
        init_2
    ]

    tri_point_3 = [
        init_3,
        init_3,
        init_3,
        init_3,
        init_3,
        (initial_gap, -initial_gap/4, 0),
        init_3
    ]

    tri_point_4 = [
        init_4,
        (far_point,near_point,0),
        (initial_gap, 0, 0),
        produit_vector(init_4, (1, 1, 0)),
        init_4,
        init_4,
        init_4
    ]

    interp1 = Interpolate(tri_point_1)
    interp3 = Interpolate(tri_point_3)
    interp2 = Interpolate(tri_point_2)
    interp4 = Interpolate(tri_point_4)
    t_1 = (t%duration)/duration
    t_2 = ((t+shift)%duration)/duration
    point1 = interp1.lerp(t_1)
    point3 = interp3.lerp(t_1)
    point2 = interp2.lerp(t_1)
    point4 = interp4.lerp(t_1)




    


    return legs([point1, point2, point3, point4])
    


    

if __name__ == "__main__":
    print("N'exécutez pas ce fichier, mais simulator.py")