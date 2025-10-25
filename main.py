#!/usr/bin/env pybricks-micropython
# Arden Feldt 740566506
# Ryder Klein 730559358

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import math

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

motorLeft = Motor(Port.A, Direction.COUNTERCLOCKWISE)
motorRight = Motor(Port.B, Direction.COUNTERCLOCKWISE)

touch = TouchSensor(Port.S1)
ultrasonic = UltrasonicSensor(Port.S2)

# Wheel radius in cm
# 2.8
WHEELRADIUS = 2.8
GOATED_SPEED = 180

taskStarted = False
# Possible states: APPROACH, FOLLOW, DONE
currentTask = "APPROACH"
ev3.screen.clear()
ev3.screen.print(currentTask)

def getAngle(dist):
    ERROR = 1.04
    dist = dist * ERROR
    circum = 2 * math.pi * WHEELRADIUS
    return (dist / circum) * 360

def goNext(newObjective):
    global taskStarted, currentTask
    motorLeft.reset_angle(0)
    motorRight.reset_angle(0)
    taskStarted = False
    currentTask = newObjective
    ev3.screen.print(currentTask)

def centerButtonPressed():
    pressed = (Button.CENTER in ev3.buttons.pressed())
    if (pressed):
        wait(100)
    return pressed

def getUsDist():
    US_OFFSET = 45
    return ultrasonic.distance() - US_OFFSET



# Write your program here.
while (True):
    '''
    Objective 1 (Detect wall): You will place your robot behind a starting line such that its
    measuring point will be on a specific starting point (marked) on the starting line (see figure
    below). You will then push the dark gray center button. Your robot should then move straight
    forward (in the positive y direction on the ground). Straight ahead, at a distance of somewhere
    between 75 and 130 cm, will be a wall that is perpendicular to the robot’s forward motion. The
    wall will be at least 17 cm high, of unknown width, and it is bumpable and can be detected via
    ultrasound.
    '''
    if (currentTask == "OBJ1"):
        if (not taskStarted):
            if (centerButtonPressed()):
                motorLeft.run(GOATED_SPEED)
                motorRight.run(GOATED_SPEED)
                taskStarted = True

        # Code to actually do the thing (should not be included in block above)
        else:
            if (touch.pressed()):
                motorLeft.brake()
                motorRight.brake()
                goNext("OBJ2")
    '''
    Objective 2 (Turn at wall): Once your robot is less than 30 centimeters from the wall, your
    robot should turn right and follow the wall.
    '''
    elif (currentTask == "OBJ2"):
        TARGET_DISTANCE = 40
        if (not taskStarted):

            taskStarted = True
        elif (getUsDist() < (TARGET_DISTANCE * 10)):
            motorLeft.hold()
            motorRight.hold()
            ev3.screen.print(getUsDist())
            goNext("OBJ31")
    elif (currentTask == "OBJ31"):
        if ((not taskStarted) and centerButtonPressed()):
            motorLeft.run(GOATED_SPEED)
            motorRight.run(GOATED_SPEED)
            taskStarted = True
        elif (touch.pressed()):
            motorLeft.hold()
            motorRight.hold()
            goNext("OBJ32")
    elif (currentTask == "OBJ32"):
        TARGET_DISTANCE = 40
        if ((not taskStarted)):
            motorLeft.run(-GOATED_SPEED)
            motorRight.run(-GOATED_SPEED)
            taskStarted = True
        if (getUsDist() > (TARGET_DISTANCE * 10)):
            motorLeft.hold()
            motorRight.hold()
            ev3.screen.print(getUsDist())
            goNext("FIN")
    wait(5)

    

