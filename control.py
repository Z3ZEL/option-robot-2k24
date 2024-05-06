import math
import numpy as np

def sandbox(t):
    """ - Entrée: t, le temps (secondes écoulées depuis le début)
        - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs"""

    # Par exemple, on envoie un mouvement sinusoidal
    targets = [0]*12
    targets[0] = 0
    targets[1] = np.pi/6
    targets[2] = -np.pi/2 + np.pi/6

    return targets

def direct(alpha, beta, gamma):
    """ - Entrées: alpha, beta, gamma, la cible (radians) des moteurs
        - Sortie: un tableau contenant la position atteinte par le bout de la patte (en mètres) """

    return [0., 0., 0.]

def inverse(x, y, z):
    """ - Entrée: x, y, z, une position cible dans le repère de la patte (mètres), provenant du slider
        - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians) """

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

# interpolate a value between p1 and p2 based on t in [0, 1]
def interpolate(p1, p2, t):
    return p1 + (p2 - p1) * t
    

def draw(t):
    """ - Entrée: t, le temps (secondes écoulées depuis le début)
        - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians) """

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
    """ - Entrée: des positions cibles (tuples (x, y, z)) pour le bout des 4 pattes
        - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs """

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
    """ - Entrée: L'angle dans lequel le triangle doit être orienté dans le plan du sol (angle 0 = axe x)
        - Sortie: Un triangle en 3D représenté par 3 points """
    triangle = [(-0.1, 0., 0), (0., 0., 0.15), (0.1, 0., 0.)]
    phi = target_angle
    A = np.array(  [ [np.cos(phi), -np.sin(phi), 0],
                     [np.sin(phi),  np.cos(phi), 0],
                     [          0,            0, 1]])
    
    newtriangle = [np.array([0.]*3)]*3
    for i in range(len(triangle)):
        newtriangle[i] = np.dot(A,triangle[i]) /2
    
    return newtriangle
    
        
# RESTING POSITIONS : Là où les pates doivent être quand le robot ne marche pas
radius = 0.2
RESTING_POS = np.array([[0.]*3]*6)
phase = np.pi/3
for i in range(6):
    RESTING_POS[i] = [radius*np.cos(phase), radius*np.sin(phase), -0.15]
    phase += np.pi/3

# ORIGINE PATES : Origine du repère de chaque pate
ORIGINE_PATES = np.array([
    [ 0.033,  0.08, 0.],
    [-0.033,  0.08, 0.],
    [-0.095,  0.00, 0.],
    [-0.033, -0.08, 0.],
    [ 0.033, -0.08, 0.],
    [ 0.095,  0.00, 0.]
])

# ANGLE_PATES : Orientation du repère de chaque pate
ANGLES_PATES = (-np.pi/2, -np.pi/2, np.pi, np.pi/2, np.pi/2, 0.)

# MATRICE ROTATION PATES : Matrice de rotation pour passer dans le repère d'une pate
MATRICES_ROTATION_PATES = []
for i in range(6):
    phi = ANGLES_PATES[i]
    A = np.array(  [ [np.cos(phi), -np.sin(phi), 0.],
                     [np.sin(phi),  np.cos(phi), 0.],
                     [          0,            0, 1.]])
    MATRICES_ROTATION_PATES.append(A)

# LEGS PHASE : Décalage (en rad) que chaque pate doit avoir quand elle effectue son motif de marche
LEGS_PHASE = [0., 1.57, 0., 1.57, 0., 1.57]

def target_angles_from_abs_pos(index_pate, pos):
    """ - Entrée:   index_pate : Index de la pate que l'on veut faire bouger 
                    pos : Position absolue (dans le repère du robot) vers laquelle on veut bouger
        - Sortie:   Les 3 positions angulaires cibles pour les moteurs de la pate """
    (x,y,z) = tuple(np.dot(MATRICES_ROTATION_PATES[index_pate], pos - ORIGINE_PATES[index_pate]))
    return inverse(x,y,z)

def rot_angle_from_pattern(speed_rotation, segment_idx, t_in_segment):
    """ - Entrée:   speed_rotation : Vitesse de rotation souhaitée du robot (unité arbitraire)
                    segment_idx : index de la face du motif de marche suivi
                    t_in_segment : avancement sur cette face du motif (dans [0, 1])
        - Sortie:   L'angle cible en rad """
    if segment_idx < 2: # phase de lever de la pate
        newt = (t_in_segment + segment_idx) / 2
        angle = interpolate(-speed_rotation, speed_rotation, newt)

    else: # phase de contact au sol
        angle = interpolate(speed_rotation, -speed_rotation, t_in_segment)
    return angle

last_segment_idx = 0
last_t_in_segment = 0
time = 0
    
def walk(t, speed_x, speed_y, speed_rotation, height = 0):
    """ - Entrée:   t : Temps depuis le début de la marche
                    speed_x, speed_y : Vitesse selon l'axe x et y souhaitée
                    speed_rotation : Vitesse de rotation souhaitée (positif = sens des aiguilles d'une montre)
        - Sortie:   Les positions angulaires cibles du robot à l'instant t """
    global time
    global last_segment_idx
    global last_t_in_segment
    delta_t = t-time

    speed_multiplier = 15. * np.sqrt(speed_x*speed_x + speed_y*speed_y)
    pattern_size = 0.6
    speed_rotation_multiplier = speed_rotation

    # détermination de la direction de la marche
    front = 0
    if speed_x > 0:
        front = np.arctan(speed_y/speed_x)
    elif speed_x < 0:
        front = np.arctan(speed_y/speed_x) + np.pi
    elif speed_y == 0:
        front = 0
    elif speed_y >= 0:
        front = np.pi/2
    else:
        front = -np.pi/2

    # création du motif pour la marche
    movement_pattern = [(0., 0., 0.)]*3
    pat = triangle(front)
    for i in range(len(pat)):
        movement_pattern[i] = pattern_size * pat[i]

    # récupération de l'avancement actuel sur le motif de marche
    base_segment_idx = (last_segment_idx + int(last_t_in_segment+speed_multiplier*delta_t)) % len(movement_pattern)
    base_t_in_segment = (last_t_in_segment+speed_multiplier*delta_t) % 1

    # calcul des nouvelles positions angulaires cibles pour chaque pate
    targets = [0.]*18
    for i in range(6):
        # calcul du nouvel avancement de la pate dans son motif de marche
        segment_idx = (base_segment_idx + int(base_t_in_segment+LEGS_PHASE[i])) % len(movement_pattern)
        t_in_segment = (base_t_in_segment+LEGS_PHASE[i]) % 1

        # calcul de la rotation à ajouter au motif pour suivre la directive speed_rotation reçue
        target_pos = [0.]*3
        phi = rot_angle_from_pattern(speed_rotation_multiplier, segment_idx, t_in_segment)
        A = np.array(  [ [np.cos(phi), -np.sin(phi), 0.],
                         [np.sin(phi),  np.cos(phi), 0.],
                         [          0,            0, 1.]])

        # calcul de la position absolue (dans le repère du robot) désirée
        target_pos = np.dot(A, RESTING_POS[i]) + interpolate(np.array(movement_pattern[segment_idx]), np.array(movement_pattern[(segment_idx+1) % len(movement_pattern)]), t_in_segment)

        # conversion de cette position absolue en position angulaires cibles pour les moteurs de la pate
        tmp = target_angles_from_abs_pos(i, target_pos)
        for j in range(3):
            targets[3*i+j] = tmp[j]

    # sauvegarde de l'avancement sur le motif de marche pour la prochaine itération
    time = t
    last_segment_idx = base_segment_idx
    last_t_in_segment = base_t_in_segment
    return targets

if __name__ == "__main__":
    print("N'exécutez pas ce fichier, mais simulator.py")