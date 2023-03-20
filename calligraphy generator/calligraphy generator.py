# -*- coding: utf-8 -*-
"""
Student names: Leonoor Verbaan, George potter, Demi
Student number: 5415721

Control in Human-Robot Interaction Assignment 3: Calligraphy generator
-------------------------------------------------------------------------------
DESCRIPTION:
Creates a simulated haptic writing device (left) and VR environment (right)

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
"""

# import pygame module in this program
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


# activate the pygame library
# initiate pygame and give permission
# to use pygame's functionality.
pygame.init()

display_surface = pygame.display.set_mode((1200, 400))  ##twice 600x400 for haptic and VR
pygame.display.set_caption('Virtual Haptic Device')

screenHaptics = pygame.Surface((600, 400))
screenVR = pygame.Surface((600, 400))

# define the RGB value for white,
#  green, blue colour .
white = (255, 255, 255)
green = (0, 255, 0)
blue = (0, 0, 128)
black = (0,0,0)
cWhite = (255, 255, 255)
cDarkblue = (36, 90, 190)
cLightblue = (0, 176, 240)
cRed = (255, 0, 0)
cOrange = (255, 100, 0)
cYellow = (255, 255, 0)

# set the pygame window name
pygame.display.set_caption('Calligraphy generator')

# create a font object.
# 1st parameter is the font file
# which is present in pygame.
# 2nd parameter is size of the font
font1 = 'BerlynaDemo-pp8r.ttf'
font2 = 'freesansbold.ttf'
font3 = 'Linotype Centennial LT 46 Light Italic.ttf'

Font = pygame.font.Font(font2, 52)

pygame.mouse.set_visible(True)  ##Hide cursor by default. 'm' toggles it


# create a text surface object,
# on which text is drawn on it.
text_gen = Font.render('Random Shit', True, black, white)

# create a rectangular object for the
# text surface object
textRect_gen = text_gen.get_rect()

# set the center of the rectangular object.
textRect_gen.center = (1200 // 4, 400 // 2)

##set up the on-screen debugToggle
text = Font.render('Virtual Haptic Device', True, (0, 0, 0), (255, 255, 255))
textRect = text.get_rect()
textRect.topleft = (10, 10)

xc, yc = screenVR.get_rect().center  ##center of the screen



##initialize "real-time" clock
clock = pygame.time.Clock()
FPS = 100  # in Hertz

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
    # Compute the force of a virtual spring where one end is attached to the center of the virtual environment and the other is connected to the position of the haptic endpoint xh.
    # Start with a spring constant that is 10x lower than the mouse-haptics spring k

    x_spring = xh - [xc,yc]
    k_spring = k/10
    fe_spring = k_spring * x_spring
    fe += fe_spring
    print(fe)

    ##Step 2 Damping and masses
    x_handle = xh - xhold

    t = time.time()
    dt = t - t_prev
    v = x_handle/dt

    b_damper = b*100
    f_damper = -b_damper * v

    # Update the force fe according to the response of the virtual environment
    fe += f_damper

    ##Step 3 Virtual shapes

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
    display_surface.blit(screenHaptics, (0, 0))
    display_surface.blit(screenVR, (600, 0))
    display_surface.blit(text_gen, textRect_gen)
    screenVR.fill(cLightblue)

    ##Print status in  overlay
    if debugToggle:
        text = font.render("FPS = " + str(round(clock.get_fps())) + \
                           "  xm = " + str(np.round(10 * xm) / 10) + \
                           "  xh = " + str(np.round(10 * xh) / 10) + \
                           "  fe = " + str(np.round(10 * fe) / 10) \
                           , True, (0, 0, 0), (255, 255, 255))
        display_surface.blit(text, textRect)

    pygame.display.flip()
    # Draws the surface object to the screen.
    pygame.display.update()
    ##Slow down the loop to match FPS
    clock.tick(FPS)

pygame.display.quit()
pygame.quit()