import os
import argparse
import math
import time
import pybullet as p
if __package__ is None or __package__ == '':
    import control
else:
    from . import control

t = 0.
dt = 0.01
dirname = os.path.dirname(__file__) + '/models/'
n_motors = 0
jointsMap = []

def init():
    """Initialise le simulateur"""
    # Instanciation de Bullet
    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -10)

    # Chargement du sol
    planeId = p.loadURDF(dirname+'/plane.urdf')

    p.setPhysicsEngineParameter(fixedTimeStep=dt)


def loadModel(name, fixed=False, startPos=[0., 0., 0.1], startOrientation=[0., 0., 0.]):
    """Charge un modèle"""
    startOrientation = p.getQuaternionFromEuler(startOrientation)
    model = p.loadURDF(dirname+"/"+name+"/robot.urdf",
                       startPos, startOrientation, useFixedBase=fixed)

    return model


def setJoints(robot, joints):
    """Définis les angles cibles pour les moteurs du robot

    Arguments:
        int -- identifiant du robot
        joints {list} -- liste des positions cibles (rad)
    """
    global jointsMap
    
    for k in range(len(joints)):
        jointInfo = p.getJointInfo(robot, jointsMap[k])
        p.setJointMotorControl2(
            robot, jointInfo[0], p.POSITION_CONTROL, joints[k])


def inverseControls(name='', x_default=0.15, y_default=0.):
    target_x = p.addUserDebugParameter(
        '%starget_x' % name, -0.3, 0.3, x_default)
    target_y = p.addUserDebugParameter(
        '%starget_y' % name, -0.3, 0.3, y_default)
    target_z = p.addUserDebugParameter('%starget_z' % name, -0.3, 0.3, 0.)
    target = loadModel('target2', True)

    return target_x, target_y, target_z, target

def directControls():
    alpha = p.addUserDebugParameter('alpha', -math.pi, math.pi, 0)
    beta = p.addUserDebugParameter('beta', -math.pi, math.pi, 0)
    gamma = p.addUserDebugParameter('gamma', -math.pi, math.pi, 0)
    target = loadModel('target2', True)

    return alpha, beta, gamma, target


def inverseUpdate(controls):
    x = p.readUserDebugParameter(controls[0])
    y = p.readUserDebugParameter(controls[1])
    z = p.readUserDebugParameter(controls[2])
    p.resetBasePositionAndOrientation(
        controls[3], [x, y, z + floatHeight], p.getQuaternionFromEuler([0, 0, 0]))

    return x, y, z

def tick():
    global t

    t += dt
    p.stepSimulation()
    time.sleep(dt)

if __name__ == "__main__":
    # Arguments
    parser = argparse.ArgumentParser(prog="Quadruped")
    parser.add_argument('-m', type=str, help='Mode', default='motors')
    parser.add_argument('-r', type=str, help='Robot', default='quadruped')
    args = parser.parse_args()
    mode = args.m
    robot = args.r

    if mode not in ['motors', 'sandbox', 'direct', 'inverse', 'draw', 'legs', 'walk']:
        print('Le mode %s est inconnu' % mode)
        exit(1)

    init()
    fixed = False
    floatHeight = 0.15

    if robot == 'quadruped':
        oneLegStartPos = [-0.04, 0., floatHeight]
        oneLegstartOrientation = [0., 0., math.pi + math.pi/4]
        n_motors = 12
        jointsMap = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
    else:
        oneLegstartOrientation = [0., 0., -math.pi/2]
        oneLegStartPos = [-0.079, 0.032, floatHeight]
        n_motors = 18
        jointsMap = list(range(18))

    startPos = [0., 0., floatHeight]
    startOrientation = [0., 0., 0.]

    if mode == 'motors':
        motors_sliders = []
        for k in range(n_motors):
            motors_sliders.append(p.addUserDebugParameter(
                "motor_%d" % k, -math.pi, math.pi, 0.))
    elif mode == 'inverse' or mode == 'direct':
        fixed = True
        startOrientation = oneLegstartOrientation
        startPos = oneLegStartPos
        leg = inverseControls() if mode == 'inverse' else directControls()
    elif mode == 'draw':
        fixed = True
        startOrientation = oneLegstartOrientation
        startPos = oneLegStartPos
        lastLine = None
    elif mode == 'legs':
        fixed = True
        legs = []
        if robot == 'quadruped':
            legs = [
                inverseControls('leg1_', -0.15, 0.15),
                inverseControls('leg2_', -0.15, -0.15),
                inverseControls('leg3_', 0.15, -0.15),
                inverseControls('leg4_', 0.15, 0.15)
            ]
        else:
            legs = [
                inverseControls('leg1_', 0.05, 0.24),
                inverseControls('leg2_', -0.05, 0.24),
                inverseControls('leg3_', -0.24, 0),
                inverseControls('leg4_', -0.05, -0.24),
                inverseControls('leg5_', 0.05, -0.24),
                inverseControls('leg6_', 0.24, 0.)
            ]       
        
    elif mode == 'walk':
        speed_x = p.addUserDebugParameter('speed_x', -0.2, 0.2, 0.)
        speed_y = p.addUserDebugParameter('speed_y', -0.2, 0.2, 0.)
        speed_rotation = p.addUserDebugParameter(
            'speed_rotation', -0.5, 0.5, 0.)
    elif mode == 'sandbox':
        print('Mode bac à sable...')

    robot = loadModel(robot, fixed, startPos, startOrientation)

    # Boucle principale
    while True:
        if mode == 'motors':
            joints = []
            for entry in motors_sliders:
                joints.append(p.readUserDebugParameter(entry))
        elif mode == 'inverse':
            joints = control.inverse(*inverseUpdate(leg)) + [0]*(n_motors-3)
        elif mode == 'direct':
            alpha_slider, beta_slider, gamma_slider, target = leg
            alpha = p.readUserDebugParameter(alpha_slider)
            beta = p.readUserDebugParameter(beta_slider)
            gamma = p.readUserDebugParameter(gamma_slider)
            joints = [alpha, beta, gamma] + [0]*(n_motors-3)
            x, y, z = control.direct(alpha, beta, gamma)
            p.resetBasePositionAndOrientation(target, [x, y, z + floatHeight],
                p.getQuaternionFromEuler([0, 0, 0]))
        elif mode == 'draw':
            joints = control.draw(t) + [0]*(n_motors-3)

            def getLegTip():
                res = p.getLinkState(robot, 3)
                return res[0]

            if lastLine is None:
                lastLine = time.time(), getLegTip()
            elif time.time() - lastLine[0] > 0.1:
                tip = getLegTip()
                p.addUserDebugLine(lastLine[1], tip, (1., 0, 0), 2., 10.)
                lastLine = time.time(), tip
        elif mode == 'legs':
            legs_xyz = [
                inverseUpdate(leg) for leg in legs
            ]

            joints = control.legs(legs_xyz)
        elif mode == 'walk':
            x = p.readUserDebugParameter(speed_x)
            y = p.readUserDebugParameter(speed_y)
            rotation = p.readUserDebugParameter(speed_rotation)
            joints = control.walk(t, x, y, rotation)
        elif mode == 'sandbox':
            joints = control.sandbox(t)

        # Envoi des positions cibles au simulateur
        setJoints(robot, joints)

        # Mise à jour de la simulation
        tick()
