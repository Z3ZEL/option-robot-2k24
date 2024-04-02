from control import walk
import time
from dynamixel_sdk import *
import math
from threading import Thread
import KBHit

portHandler = PortHandler("/dev/ttyACM0")
packetHandler = PacketHandler(1.0)

portHandler.openPort()
portHandler.setBaudRate(1000000)

ADDR_GOAL_POSITION = 30
PRESENT_POSITION = 36

# ids = []
# for id in range(0, 100):
#     model_number, result, error = packetHandler.ping(portHandler, id)
#     if model_number == 12:
#         print(f"Found AX-12 with id: {id}")
#         ids.append(id)
#     time.sleep(0.01)

# print("Found motors:")
# print(ids)

print("Send motors to zero (press enter)")
input()
UPDATE_FREQ = 0.1

isPaused = False
t = 0
t0 = time.time()


correspondance_table = [
    1,
    3,
    5,

    2,
    4,
    6,

    8,
    10,
    12,

    14,
    16,
    18,

    13,
    15,
    17,

    7,
    9,
    11
]

def radToDeg(rad):
    return rad * 180 / 3.14

def degAngleToPos(angle):
    return int((angle+150) * 1023 / 300)

def radAngleToPos(angle):
    return degAngleToPos(radToDeg(angle))

def nearlyEqual(a, b, epsilon):
    return abs(a - b) < epsilon

kb = KBHit.KBHit()

def setPositions(ids, positions):
    assert len(ids) == len(positions)

    currentAttempt = 0
    isSet = [0]*len(ids)

    while 0 in isSet:
        if(isSet[currentAttempt%len(ids)] == 1):
            currentAttempt += 1
            continue
        if(nearlyEqual(packetHandler.read2ByteTxRx(portHandler,correspondance_table[ids[currentAttempt%len(ids)]], PRESENT_POSITION)[0],positions[currentAttempt%len(ids)],10)):
            isSet[currentAttempt%len(ids)] = 1
        else:
            packetHandler.write2ByteTxRx(portHandler,correspondance_table[ids[currentAttempt%len(ids)]], ADDR_GOAL_POSITION, positions[currentAttempt%len(ids)])
        currentAttempt += 1

def reset():
    print("Resetting")
    setPositions([0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17], [512]*18)
    print("Reset done")



while 1:
    #READ INPUTS BUFFER NO BLOCKING
    if kb.kbhit():
        c = kb.getch()
        if c == 'p':
            isPaused = not isPaused
            if isPaused:
                print("Paused, press p to resume")
        if c == 'r':
            reset()
        if c == 'n':
            reset()
            ids = [0,12,16,17]
            positions = [degAngleToPos(-50), degAngleToPos(50), degAngleToPos(-45), degAngleToPos(90)]
            setPositions(ids, positions)
        if c == 'c':
            reset()
            ids = [1,2,3,4,5,6,7,8,9,10,11,13,14,15,16,17]
            setPositions(ids, [0]*16)


        elif c == 'q':
            break
        

        

    if not isPaused:
        # Compute time since last iteration
        t1 = time.time()
        dt = t1 - t0
        t0 = t1
        t += dt


        # Compute the new position of the robot
        angles = walk(t, 0.1, 0.1, 0)

        mapped_angles = list(map(lambda x: radAngleToPos(-x), angles))
        
        
        # degAngle=math.cos(t)*90+180
        # packetHandler.write2ByteTxRx(portHandler, correspondance_table[2], ADDR_GOAL_POSITION, degAngleToPos(degAngle))
        # for j in range(0,18):
        #     if(j == 2):
        #         continue
        #     packetHandler.write2ByteTxRx(portHandler, correspondance_table[j], ADDR_GOAL_POSITION, 512)

        # setPositions([0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17], mapped_angles)

        for i in range(0, 18):
            packetHandler.write2ByteTxRx(portHandler, correspondance_table[i], ADDR_GOAL_POSITION, mapped_angles[i])

        time.sleep(UPDATE_FREQ)
        print(angles)
        print(mapped_angles)
    else:
        time.sleep(UPDATE_FREQ)