# -*- coding: utf-8 -*-
"""
Student name: Leonoor Verbaan
Student number: 5415721

Control in Human-Robot Interaction Assignment 1: Haptic Rendering
-------------------------------------------------------------------------------
DESCRIPTION:
Creates a simulated haptic device (left) and VR environment (right)

The forces on the virtual haptic device are displayed using pseudo-haptics. The
code uses the mouse as a reference point to simulate the "position" in the
user's mind and couples with the virtual haptic device via a spring. the
dynamics of the haptic device is a pure damper, subjected to perturbations
from the VR environment.

IMPORTANT VARIABLES
xc, yc -> x and y coordinates of the center of the haptic device and of the VR
xm -> x and y coordinates of the mouse cursor
xh -> x and y coordinates of the haptic device (shared between real and virtual panels)
fe -> x and y components of the force fedback to the haptic device from the virtual impedances

TASKS:
1- Implement the impedance control of the haptic device
2- Implement an elastic element in the simulated environment
3- Implement a position dependent potential field that simulates a bump and a hole
4- Implement the collision with a 300x300 square in the bottom right corner
5- Implement the god-object approach and compute the reaction forces from the wall

REVISIONS:
Initial release MW - 14/01/2021
Added 2 screens and Potential field -  21/01/2021
Added Collision and compressibility (LW, MW) - 25/01/2021
Added Haptic device Robot (LW) - 08/02/2022

INSTRUCTORS: Michael Wiertlewski & Laurence Willemet & Mostafa Attala
e-mail: {m.wiertlewski,l.willemet,m.a.a.atalla}@tudelft.nl
"""

import pygame
import numpy as np
import math
import matplotlib.pyplot as plt
from pantograph import Pantograph
from pyhapi import Board, Device, Mechanisms
from pshape import PShape
import sys, serial, glob
from serial.tools import list_ports
import time

##################### General Pygame Init #####################
##initialize pygame window
pygame.init()
window = pygame.display.set_mode((1200, 400))  ##twice 600x400 for haptic and VR
pygame.display.set_caption('Virtual Haptic Device')

screenHaptics = pygame.Surface((600, 400))
screenVR = pygame.Surface((600, 400))

##add nice icon from https://www.flaticon.com/authors/vectors-market
icon = pygame.image.load('robot.png')
pygame.display.set_icon(icon)

##add text on top to debugToggle the timing and forces
font = pygame.font.Font('freesansbold.ttf', 18)

pygame.mouse.set_visible(True)  ##Hide cursor by default. 'm' toggles it

##set up the on-screen debugToggle
text = font.render('Virtual Haptic Device', True, (0, 0, 0), (255, 255, 255))
textRect = text.get_rect()
textRect.topleft = (10, 10)

xc, yc = screenVR.get_rect().center  ##center of the screen

##initialize "real-time" clock
clock = pygame.time.Clock()
FPS = 100  # in Hertz

##define some colors
cWhite = (255, 255, 255)
cDarkblue = (36, 90, 190)
cLightblue = (0, 176, 240)
cRed = (255, 0, 0)
cOrange = (255, 100, 0)
cYellow = (255, 255, 0)

##################### Init Simulated haptic device #####################

'''*********** Student should fill in ***********'''
####Virtual environment -  Wall

####Virtual environment -  Force fieldToggle f(x,y)


##Compute the height map and the gradient along x and y
z_bump = 1000
z_hole = -1000
x, y = np.meshgrid(np.arange(400), np.arange(600))
mx_bump, my_bump = 150, 150
mx_hole, my_hole = 300, 400
sx, sy = 50, 50


gaussian_bump = z_bump * np.exp(-((x - mx_bump) ** 2 / (2 * sx ** 2) + (y - my_bump) ** 2 / (2 * sy ** 2)))
gradient_map_bump = np.array(np.gradient(gaussian_bump))

gaussian_hole = z_hole * np.exp(-((x - mx_hole) ** 2 / (2 * sx ** 2) + (y - my_hole) ** 2 / (2 * sy ** 2)))
gradient_map_hole = np.array(np.gradient(gaussian_hole))

'''*********** !Student should fill in ***********'''

####Pseudo-haptics dynamic parameters, k/b needs to be <1
k = .5  ##Stiffness between cursor and haptic display
b = .8  ##Viscous of the pseudohaptic display

##################### Define sprites #####################

##define sprites
hhandle = pygame.image.load('handle.png')
haptic = pygame.Rect(*screenHaptics.get_rect().center, 0, 0).inflate(48, 48)
cursor = pygame.Rect(0, 0, 5, 5)
colorHaptic = cOrange  ##color of the wall

xh = np.array(haptic.center)

##Set the old value to 0 to avoid jumps at init
xhold = 0
xmold = 0

##################### Init Virtual env. #####################

'''*********** !Student should fill in ***********'''
##hint use pygame.rect() to create rectangles

'''*********** !Student should fill in ***********'''


##################### Detect and Connect Physical device #####################
# USB serial microcontroller program id data:
def serial_ports():
    """ Lists serial port names """
    ports = list(serial.tools.list_ports.comports())

    result = []
    for p in ports:
        try:
            port = p.device
            s = serial.Serial(port)
            s.close()
            if p.description[0:12] == "Arduino Zero":
                result.append(port)
                print(p.description[0:12])
        except (OSError, serial.SerialException):
            pass
    return result


CW = 0
CCW = 1

haplyBoard = Board
device = Device
SimpleActuatorMech = Mechanisms
pantograph = Pantograph
robot = PShape

#########Open the connection with the arduino board#########
port = serial_ports()  ##port contains the communication port or False if no device
if port:
    print("Board found on port %s" % port[0])
    haplyBoard = Board("test", port[0], 0)
    device = Device(5, haplyBoard)
    pantograph = Pantograph()
    device.set_mechanism(pantograph)

    device.add_actuator(1, CCW, 2)
    device.add_actuator(2, CW, 1)

    device.add_encoder(1, CCW, 241, 10752, 2)
    device.add_encoder(2, CW, -61, 10752, 1)

    device.device_set_parameters()
else:
    print("No compatible device found. Running virtual environnement...")
    # sys.exit(1)

# conversion from meters to pixels
window_scale = 3

##################### Main Loop #####################
##Run the main loop
##TODO - Perhaps it needs to be changed by a timer for real-time see:
##https://www.pygame.org/wiki/ConstantGameSpeed

run = True
ongoingCollision = False
fieldToggle = True
robotToggle = True
wallToggle = False
gradToggle = False

debugToggle = False

while run:

    #########Process events  (Mouse, Keyboard etc...)#########
    for event in pygame.event.get():
        ##If the window is close then quit
        if event.type == pygame.QUIT:
            run = False
        elif event.type == pygame.KEYUP:
            if event.key == ord('m'):  ##Change the visibility of the mouse
                pygame.mouse.set_visible(not pygame.mouse.get_visible())
            if event.key == ord('q'):  ##Force to quit
                run = False
            if event.key == ord('d'):
                debugToggle = not debugToggle
            if event.key == ord('r'):
                robotToggle = not robotToggle
            '''*********** Student can add more ***********'''
            ##Toggle the wall or the height map
            if event.key == ord('w'):
                wallToggle = not wallToggle
            if event.key == ord('g'):
                gradToggle = not gradToggle

            '''*********** !Student can add more ***********'''

    ######### Read position (Haply and/or Mouse)  #########

    ##Get endpoint position xh
    if port and haplyBoard.data_available():  ##If Haply is present
        # Waiting for the device to be available
        #########Read the motorangles from the board#########
        device.device_read_data()
        motorAngle = device.get_device_angles()

        #########Convert it into position#########
        device_position = device.get_device_position(motorAngle)
        xh = np.array(device_position) * 1e3 * window_scale
        xh[0] = np.round(-xh[0] + 300)
        xh[1] = np.round(xh[1] - 60)
        xm = xh  ##Mouse position is not used

    else:
        ##Compute distances and forces between blocks
        xh = np.clip(np.array(haptic.center), 0, 599)
        xh = np.round(xh)

        ##Get mouse position
        cursor.center = pygame.mouse.get_pos()
        xm = np.clip(np.array(cursor.center), 0, 599)

    '''*********** Student should fill in ***********'''

    fe = np.zeros(2)  ##Environment force is set to 0 initially.
    t_prev = 0

    ##Replace

    ######### Compute forces ########

    ##Step 1 Elastic impedance

    ##Step 2 Damping and masses

    ##Step 3 Virtual shapes

    #Bump
    grad_dx_bump = gradient_map_bump[0][xh[0]][xh[1]]
    grad_dy_bump = gradient_map_bump[1][xh[0]][xh[1]]
    k_shape = k
    f_shape_bump = k_shape * np.array([grad_dx_bump, grad_dy_bump]) #Spring element

    #Hole
    grad_dx_hole = gradient_map_hole[0][xh[0]][xh[1]]
    grad_dy_hole = gradient_map_hole[1][xh[0]][xh[1]]
    k_shape = k
    f_shape_hole = k_shape * np.array([grad_dx_hole, grad_dy_hole]) #Spring element

    #Update the force fe according to the response of the virtual environment
    fe += f_shape_bump
    fe += f_shape_hole

    ##Step 4 Virtual wall


    ##Step 5 Bonus: Friction

    '''*********** !Student should fill in ***********'''

    ######### Send forces to the device #########
    if port:
        fe[1] = -fe[1]  ##Flips the force on the Y=axis

        ##Update the forces of the device
        device.set_device_torques(fe)
        device.device_write_torques()
        # pause for 1 millisecond
        time.sleep(0.001)
    else:
        ######### Update the positions according to the forces ########
        ##Compute simulation (here there is no inertia)
        ##If the haply is connected xm=xh and dxh = 0
        dxh = (k / b * (
                    xm - xh) / window_scale - fe / b)  ####replace with the valid expression that takes all the forces into account
        dxh = dxh * window_scale
        xh = np.round(xh + dxh)  ##update new positon of the end effector

    haptic.center = xh

    ##Update old samples for velocity computation
    xhold = xh
    xmold = xm

    ######### Graphical output #########
    ##Render the haptic surface
    screenHaptics.fill(cWhite)

    ##Change color based on effort
    colorMaster = (255, \
                   255 - np.clip(np.linalg.norm(k * (xm - xh) / window_scale) * 15, 0, 255), \
                   255 - np.clip(np.linalg.norm(k * (xm - xh) / window_scale) * 15, 0,
                                 255))  # if collide else (255, 255, 255)

    pygame.draw.rect(screenHaptics, colorMaster, haptic, border_radius=4)

    ######### Robot visualization ###################
    # update individual link position
    if robotToggle:
        robot.createPantograph(screenHaptics, xh)

    ### Hand visualisation
    screenHaptics.blit(hhandle, (haptic.topleft[0], haptic.topleft[1]))
    pygame.draw.line(screenHaptics, (0, 0, 0), (haptic.center), (haptic.center + 2 * k * (xm - xh)))

    ##Render the VR surface
    # screenVR.fill(cLightblue)
    '''*********** Student should fill in ***********'''
    ### here goes the visualisation of the VR sceen.
    ### Use pygame.draw.rect(screenVR, color, rectangle) to render rectangles.

    pygame.draw.rect(screenVR, colorHaptic, haptic)

    '''*********** !Student should fill in ***********'''

    ##Fuse it back together
    window.blit(screenHaptics, (0, 0))
    window.blit(screenVR, (600, 0))
    screenVR.fill(cLightblue)

    ##Print status in  overlay
    if debugToggle:
        text = font.render("FPS = " + str(round(clock.get_fps())) + \
                           "  xm = " + str(np.round(10 * xm) / 10) + \
                           "  xh = " + str(np.round(10 * xh) / 10) + \
                           "  fe = " + str(np.round(10 * fe) / 10) \
                           , True, (0, 0, 0), (255, 255, 255))
        window.blit(text, textRect)

    pygame.display.flip()
    ##Slow down the loop to match FPS
    clock.tick(FPS)

pygame.display.quit()
pygame.quit()

