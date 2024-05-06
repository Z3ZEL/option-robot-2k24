from control import walk, inverse
import time
from dynamixel_sdk import *
import math
from threading import Thread
import KBHit
import sys
import pyautogui
from pynput import keyboard

portHandler = PortHandler("/dev/ttyACM0")
packetHandler = PacketHandler(1.0)

portHandler.openPort()
portHandler.setBaudRate(1000000)

ADDR_GOAL_POSITION = 30
PRESENT_POSITION = 36


UPDATE_FREQ = 0.00001


inverseMode = len(sys.argv) > 1 and sys.argv[1] == "inverse"


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

def setPositions(ids, positions, noCheck=False):
    assert len(ids) == len(positions)

    if(noCheck):
        for i in range(0, len(ids)):
            packetHandler.write2ByteTxRx(portHandler,correspondance_table[ids[i]], ADDR_GOAL_POSITION, positions[i])
        return

    currentAttempt = 0 
    isSet = [0]*len(ids)

    while (0 in isSet) and currentAttempt < 100:
        
        if(isSet[currentAttempt%len(ids)] == 1):
            currentAttempt += 1
            continue
        if(nearlyEqual(packetHandler.read2ByteTxRx(portHandler,correspondance_table[ids[currentAttempt%len(ids)]], PRESENT_POSITION)[0],positions[currentAttempt%len(ids)],20)):
            isSet[currentAttempt%len(ids)] = 1
        else:
            packetHandler.write2ByteTxRx(portHandler,correspondance_table[ids[currentAttempt%len(ids)]], ADDR_GOAL_POSITION, positions[currentAttempt%len(ids)])
        currentAttempt += 1

def reset():
    print("Resetting")
    setPositions([0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17], [512]*18)
    print("Reset done")


x, y, z = 0.150, 0, 0



def main():
    speed_x,speed_y = 0.02,0.02
    speed_rotation = 0
    lastMousePos = pyautogui.position()
    t = 0
    t0 = time.time()
    isPaused = False
    mov_key_pressed = False

    while 1:
        
        if inverseMode:
            ## COMPUTE (x,y,z) from mouse position (starting from 0,0)
            
            currentMousePos = pyautogui.position()
            x += (currentMousePos[0] - lastMousePos[0]) / 1000
            y += (currentMousePos[1] - lastMousePos[1]) / 1000
            lastMousePos = currentMousePos

            
            print(f"x: {x}, y: {y}")

            
            ## COMPUTE ANGLES
            targets = list(map(lambda x: radAngleToPos(-x), inverse(x, y, z)))
            print(targets)
            ## SET ANGLES
            setPositions([0,1,2], targets, noCheck=True)
            time.sleep(0.1)
            continue

        
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
            if c == 'v':
                #ask for new speed vector
                print("Enter new speed vector")
                print("Enter x speed")
                speed_x = float(input())
                print("You selected x = ", speed_x, "Enter y speed")
                speed_y = float(input())
                print("You selected y = ", speed_y)


            if c == "z" or c == "s" or c == "q" or c == "d":
                print("Walking...")
                mov_key_pressed = time.time()
                if c == "q":
                    speed_rotation = 0.2
                if c == "d":
                    speed_rotation = -0.2

        else:
            if time.time() - mov_key_pressed > 0.3:
                mov_key_pressed = False
            

        if not isPaused:
            # Compute time since last iteration
            t1 = time.time()
            dt = t1 - t0
            t0 = t1
            t += dt


            # Compute the new position of the robotd
            angles = []
            
            if mov_key_pressed:
                print("Walking")
                angles = walk(t, speed_x, speed_y, speed_rotation)
            else:
                angles = walk(t, 0, 0, 0,0)


            mapped_angles = list(map(lambda x: radAngleToPos(-x), angles))
            
            for i in range(0, 18):
                packetHandler.write2ByteTxOnly(portHandler, correspondance_table[i], ADDR_GOAL_POSITION, mapped_angles[i])
            

            # print('Time to send motors: ', time.time() - t1)
            time.sleep(UPDATE_FREQ)
        else:
            time.sleep(UPDATE_FREQ)

if __name__ == "__main__":
    main()
    kb.set_normal_term()
    portHandler.closePort()
    print("Exiting...")