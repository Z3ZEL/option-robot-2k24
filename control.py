import math
import numpy as np

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
    targets[0] = 0
    targets[1] = np.pi/6
    targets[2] = -np.pi/2 + np.pi/6

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

    return [0., 0., 0.]

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
    l1 = 42.271e-3 #0.049
    l2 = 0.065
    l3 = 0.095
    origin_offset = 30.942e-3

    z+=origin_offset

    
    alpha = math.atan2(y, x)
    
    x_a = l1*math.cos(alpha)
    y_a = l1*math.sin(alpha)
    
    D = math.sqrt((x - x_a)**2 + (y - y_a)**2 + z**2)

    beta = 0
    if (l2 != 0 and D!=0):
        beta = math.acos( np.clip((l2**2 + D**2 - l3**2)/(2*D*l2), -1, 1)) + math.asin(z/D)

    if (l2!=0 and l3!=0):
        gamma = math.acos( np.clip( (D**2-l2**2 -l3**2)/(2*l2*l3), -1, 1))

    # hexapod version
    alpha = -alpha

    tmp_angle = np.pi/6

    PHASE = np.array([0, tmp_angle, -np.pi/2+tmp_angle+np.pi/30])

    res = np.array([alpha, beta, gamma]) 
    res += PHASE

    return list(res)

def interpolate(p1, p2, t):
    return p1 + (p2 - p1) * t
    

def draw(t):
    """
    python simulator.py -m draw

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Le but est, à partir du temps donné, de suivre une trajectoire de triangle. Pour ce faire, on
    utilisera une interpolation linéaire entre trois points, et la fonction inverse précédente.

    - Entrée: t, le temps (secondes écoulées depuis le début)
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """

    key_points = [(0.1, -0.1, 0), (0.1, 0., 0.1), (0.1, 0.1, 0.)]
    
    # Perform linear interpolation between the key points
    # Find the appropriate segment based on time
    segment_idx = int(t) % len(key_points)
    t_in_segment = t % 1

    # Calculate the target positions
    target_positions = interpolate(np.array(key_points[segment_idx]), np.array(key_points[(segment_idx+1) % len(key_points)]), t_in_segment)

    (x, y, z) = target_positions
    return inverse(x, y, z)

def legs(targets_robot):
    """
    python simulator.py -m legs

    Le robot est figé en l'air, on contrôle toute les pattes

    - Sliders: les 12 coordonnées (x, y, z) du bout des 4 pattes
    - Entrée: des positions cibles (tuples (x, y, z)) pour le bout des 4 pattes
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """

    # positions in legs bases
    leg_based_coordinates = [(0.0, 0., 0.)]*6

    POS_PATES = np.array([[0.]*3]*6)

    POS_PATES = np.array([
        [ 0.033,  0.08, 0.],
        [-0.033,  0.08, 0.],
        [-0.095,  0.00, 0.],
        [-0.033, -0.08, 0.],
        [ 0.033, -0.08, 0.],
        [ 0.095,  0.00, 0.]
    ])

    ANGLES_PATES = (0., 0., 3*np.pi/2, np.pi, np.pi, np.pi/2)

    offset = -np.pi/2 #targets_robot[0][0]*8*np.pi
    for i in range(6):
        phi = ANGLES_PATES[i] + offset
        A = np.array(  [ [np.cos(phi), -np.sin(phi), 0.],
                         [np.sin(phi),  np.cos(phi), 0.],
                         [          0,            0, 1.]])

        leg_based_coordinates[i] = tuple(np.dot(A, targets_robot[i] - POS_PATES[i]))
        

    print();

    targets = [0]*18

    for i in range(6):
        (x, y, z) = leg_based_coordinates[i]
        tmp = inverse(x, y, z)
        for j in range(3):
            targets[3*i+j] = tmp[j]

    return targets



def triangle(target_angle):
    triangle = [(-0.1, 0., 0), (0., 0., 0.15), (0.1, 0., 0.)]
    phi = target_angle
    A = np.array(  [ [np.cos(phi), -np.sin(phi), 0],
                     [np.sin(phi),  np.cos(phi), 0],
                     [          0,            0, 1]])
    
    newtriangle = [np.array([0.]*3)]*3
    for i in range(len(triangle)):
        newtriangle[i] = np.dot(A,triangle[i]) /2
    
    return newtriangle
    
        
# RESTING POSITIONS 
radius = 0.2
RESTING_POS = np.array([[0.]*3]*6)
phase = np.pi/3
for i in range(6):
    RESTING_POS[i] = [radius*np.cos(phase), radius*np.sin(phase), -0.15]
    phase += np.pi/3

POS_PATES = np.array([
    [ 0.033,  0.08, 0.],
    [-0.033,  0.08, 0.],
    [-0.095,  0.00, 0.],
    [-0.033, -0.08, 0.],
    [ 0.033, -0.08, 0.],
    [ 0.095,  0.00, 0.]
])

ANGLES_PATES = (0., 0., 3*np.pi/2, np.pi, np.pi, np.pi/2)

def walk(t, speed_x, speed_y, speed_rotation):
    """
    python simulator.py -m walk

    Le but est d'intégrer tout ce que nous avons vu ici pour faire marcher le robot

    - Sliders: speed_x, speed_y, speed_rotation, la vitesse cible du robot
    - Entrée: t, le temps (secondes écoulées depuis le début)
            speed_x, speed_y, et speed_rotation, vitesses cibles contrôlées par les sliders
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """
    speed_multiplier = 300. * speed_x #15. * np.sqrt(speed_x*speed_x + speed_y*speed_y)
    targets = [0]*18
    legs_phase = [0., 1.57, 0., 1.57, 0., 1.57]
    target_angle = speed_rotation #np.arctan(speed_y/(speed_x+0.000000001))
    pattern_size = 20 * speed_y

    tmp = triangle(target_angle)

    movement_pattern = [(0., 0., 0.)]*3
    for i in range(len(tmp)):
        movement_pattern[i] = pattern_size * tmp[i]

    # positions in legs bases
    leg_based_coordinates = [(0., 0., 0.)]*6


    offset = -np.pi/2

    for i in range(6):
        phi = ANGLES_PATES[i] + offset

        segment_idx = int(t*speed_multiplier+legs_phase[i]) % len(movement_pattern)
        t_in_segment = (t*speed_multiplier+legs_phase[i]) % 1

        target_position = RESTING_POS[i] + interpolate(np.array(movement_pattern[segment_idx]), np.array(movement_pattern[(segment_idx+1) % len(movement_pattern)]), t_in_segment)

        (x, y, z) = target_position

        A = np.array(  [ [np.cos(phi), -np.sin(phi), 0.],
                         [np.sin(phi),  np.cos(phi), 0.],
                         [          0,            0, 1.]])

        leg_based_coordinates[i] = tuple(np.dot(A, target_position - POS_PATES[i]))
    

    for i in range(6):
        (x, y, z) = leg_based_coordinates[i]
        tmp = inverse(x, y, z)
        for j in range(3):
            targets[3*i+j] = tmp[j]


    return targets

if __name__ == "__main__":
    print("N'exécutez pas ce fichier, mais simulator.py")