#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# This file presents an interface for interacting with the Playstation 4 Controller
# in Python. Simply plug your PS4 controller into your computer using USB and run this
# script!
#
# NOTE: I assume in this script that the only joystick plugged in is the PS4 controller.
#       if this is not the case, you will need to change the class accordingly.
#
# Copyright © 2015 Clay L. McLeod <clay.l.mcleod@gmail.com>
#
# Distributed under terms of the MIT license.

import os
import pprint
import time
import pygame
from control import walk
import robot_controller
import math
import numpy

dead_zone=0.1


class PS4Controller(object):
    """Class representing the PS4 controller. Pretty straightforward functionality."""

    controller = None
    axis_data = None
    button_data = None
    hat_data = None

    def init(self):
        """Initialize the joystick components"""
        
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

    def listen(self):
        """Listen for events to happen"""
        
        if not self.axis_data:
            self.axis_data = {}

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)

        t = 0
        t0 = time.time()
        speed_rotation = 0
        max_speed_x = 0.3
        max_speed_y = 0.3
        max_speed_rotation=0.2
        height = 0.04
        paused=False
        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    self.axis_data[event.axis] = round(event.value,2)
                elif event.type == pygame.JOYBUTTONDOWN:
                    self.button_data[event.button] = True
                elif event.type == pygame.JOYBUTTONUP:
                    self.button_data[event.button] = False
                elif event.type == pygame.JOYHATMOTION:
                    self.hat_data[event.hat] = event.value

                # Insert your code on what you would like to happen for each event here!
                # In the current setup, I have the state simply printing out to the screen.
                
                os.system('clear')

                
                speed_x=0
                speed_y=0
                speed_rotation=0
                current_height=0

                # print(self.axis_data)

                if(0 in self.axis_data and abs(self.axis_data[0]) > dead_zone):
                    f = self.axis_data[0]
                    f*=-1
                    speed_y = f*max_speed_y 

                if(1 in self.axis_data and abs(self.axis_data[1]) > dead_zone):
                    f = self.axis_data[1]
                    f*=-1
                    speed_x = f*max_speed_x


                if(3 in self.axis_data and abs(self.axis_data[3]) > dead_zone):
                    f = self.axis_data[3]
                    f*=-1
                    speed_rotation = f*max_speed_rotation

                # if(4 in self.axis_data and abs(self.axis_data[4]) > dead_zone):
                #     f = self.axis_data[4]
                #     current_height = f*height


                        
                # Compute time since last iteration
                t1 = time.time()
                dt = t1 - t0
                t0 = t1
                t += dt


                # Compute the new position of the robot
                angles = []

                
                # print("Speed x = ", speed_x, "Speed y = ", speed_y, "Speed rotation = ", speed_rotation)
                print(t)
                angles = walk(t, speed_x, speed_y, speed_rotation, current_height)
                


                mapped_angles = list(map(lambda x: robot_controller.radAngleToPos(-x), angles))
                
                for i in range(0, 18):
                    robot_controller.packetHandler.write2ByteTxOnly(robot_controller.portHandler, robot_controller.correspondance_table[i], robot_controller.ADDR_GOAL_POSITION, mapped_angles[i])
                


if __name__ == "__main__":
    ps4 = PS4Controller()
    ps4.init()
    ps4.listen()