# Arden Feldt 740566506
# Ryder Klien 730559358

#!/usr/bin/env pybricks-micropython
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
previously_pressed = False
# taskFinished = False
# Possible states: OBJ1, OBJ2, OBJ31, OBJ32, FIN
currentTask = "OBJ1"
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
    global previously_pressed
    pressed = Button.CENTER in ev3.buttons.pressed()
    if pressed and not previously_pressed:
        previously_pressed = True
        return True
    elif not pressed:
        previously_pressed = False
    return False
#    return (Button.CENTER in ev3.buttons.pressed())
def getUSDist():
    US_OFFSET = 45
    return ultrasonic.distance() - US_OFFSET



# Write your program here.
while (True):
    if (currentTask == "OBJ1"):
        if (not taskStarted):
            DISTANCE_ONE = 140
            motorLeft.run_angle(GOATED_SPEED, getAngle(DISTANCE_ONE), then=Stop.HOLD, wait=False)
            motorRight.run_angle(GOATED_SPEED, getAngle(DISTANCE_ONE), then=Stop.HOLD, wait=True)
            taskStarted = True
        #     continue
        # if (motorLeft.speed() == 0 and motorRight.speed() == 0):
            goNext("OBJ2")
    elif (currentTask == "OBJ2"):
        TARGET_DISTANCE = 40
        if ((not taskStarted) and centerButtonPressed()):
            motorLeft.run(GOATED_SPEED)
            motorRight.run(GOATED_SPEED)
            taskStarted = True
        if (getUSDist() < (TARGET_DISTANCE * 10)):
            motorLeft.hold()
            motorRight.hold()
            ev3.screen.print(getUSDist())
            goNext("OBJ31")
    elif (currentTask == "OBJ31"):
        if ((not taskStarted) and centerButtonPressed()):
            motorLeft.run(GOATED_SPEED)
            motorRight.run(GOATED_SPEED)
            taskStarted = True
        if (touch.pressed()):
            motorLeft.hold()
            motorRight.hold()
            goNext("OBJ32")
    elif (currentTask == "OBJ32"):
        TARGET_DISTANCE = 40
        if ((not taskStarted)):
            motorLeft.run(-GOATED_SPEED)
            motorRight.run(-GOATED_SPEED)
            taskStarted = True
        if (getUSDist() > (TARGET_DISTANCE * 10)):
            motorLeft.hold()
            motorRight.hold()
            ev3.screen.print(getUSDist())
            goNext("FIN")
    wait(5)

    

