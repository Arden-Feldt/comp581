#!/usr/bin/env pybricks-micropython
# Arden Feldt 730566506
# Ryder Klein 730559358

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.tools import wait
from pybricks.media.ev3dev import Font

import math

# Objects
ev3 = EV3Brick()

bigfont = Font(size=24)
ev3.screen.set_font(bigfont)

motorLeft = Motor(Port.A, Direction.COUNTERCLOCKWISE)
motorRight = Motor(Port.B, Direction.COUNTERCLOCKWISE)

gyro = GyroSensor(Port.S1)
ultrasonic = UltrasonicSensor(Port.S2)
touch1 = TouchSensor(Port.S3)
touch2 = TouchSensor(Port.S4)


# Constants
# Wheel radius in cm
# 2.8 nominally, subtract 5 mm for compression
WHEELRADIUS = 2.75
GOATED_SPEED = 160
WHEEL_BASE = 9
STARTINGX = 50
STARTINGY = 0
LASTHITX = 0
LASTHITY = 0
# path we wanna follow : y = 1.25 * x - 62.5


# Variables
# Task management
taskStarted = False
# Possible states: APPROACH, FOLLOW, DONE
currentTask = "APPROACH"

# Dead reckoning
x = 0
y = 0
theta = 0
totalDistance = 0
prevLeft = 0
prevRight = 0
prevGyro = 0
hitObject = False
facingObjective = False

lastDistance = 0
ev3.screen.clear()

def touchPressed():
    if touch1.pressed() or touch2.pressed():
        return True
    return False

def getAngle(dist):
    # 1.4
    ERROR = 1.0
    dist = dist * ERROR
    circum = 2 * math.pi * WHEELRADIUS
    return (dist / circum) * 360

def goNext(newObjective):
    global taskStarted, currentTask
    motorLeft.reset_angle(0)
    motorRight.reset_angle(0)
    taskStarted = False
    currentTask = newObjective

def centerButtonPressed():
    pressed = (Button.CENTER in ev3.buttons.pressed())
    if (pressed):
        wait(100)
    return pressed

def runMotors(speedLeft, speedRight):
    motorLeft.run(speedLeft)
    motorRight.run(speedRight)

def turnRightish(angleDeg, speed=100):
    arcLength = math.pi * WHEEL_BASE * angleDeg / 360
    wheelRotation = (arcLength / (2 * math.pi * WHEELRADIUS)) * 360
    motorLeft.run_angle(speed, wheelRotation, then=Stop.HOLD, wait=False)
    motorRight.run_angle(-speed, wheelRotation, then=Stop.HOLD, wait=True)

def getUpdatedPosition(motorLeft, motorRight, gyro):
    global x, y, theta, totalDistance, prevLeft, prevRight, prevGyro

    angleLeft = motorLeft.angle()
    angleRight = motorRight.angle()
    angleGyro = math.radians(gyro.angle())

    # Wheel distances
    deltaLeft = math.radians(angleLeft - prevLeft) * (WHEELRADIUS)
    deltaRight = math.radians(angleRight - prevRight) * (WHEELRADIUS)
    deltaAvg = (deltaLeft + deltaRight) / 2
    totalDistance += deltaAvg

    # Heading
    deltaTheta = angleGyro - prevGyro
    theta += deltaTheta

    # x, y
    x += deltaAvg * math.cos(theta)
    y += deltaAvg * math.sin(theta)

    # Save for next loop
    prevLeft = angleLeft
    prevRight = angleRight
    prevGyro = angleGyro

    return x, y, theta

def onMLine():
    global x, y
    if (abs((1.25 * x - 62.5) - y) < 5):
        return True
    return False

def dist_to_goal(x, y, goalX=250, goalY=250):
    dx = goalX - x
    dy = goalY - y
    return math.sqrt(dx*dx + dy*dy)

def turnTowardPoint(targetX, targetY, speed=100):
    global x, y, theta

    desiredTheta = math.atan2(targetY - y, targetX - x)
    angleError = desiredTheta - theta

    angleError = (angleError + math.pi) % (2 * math.pi) - math.pi

    angleDeg = math.degrees(angleError)

    turnRightish(angleDeg, speed)
    # if angleDeg > 0:
    #     turnRightish(angleDeg, speed)
    # else:
    #     turnRightish(angleDeg, speed)   # this is neg so it would actually go left!


# Main loop
while (True):
    if (not facingObjective):
        turnTowardPoint(250, 250)
        facingObjective = True

    if (onMLine() and facingObjective):
        runMotors(GOATED_SPEED, GOATED_SPEED)
        if (touchPressed()):
                LASTHITX = x
                LASTHITY = y
                BACKTRACK = -2
                motorLeft.run_angle(GOATED_SPEED, getAngle(BACKTRACK), then=Stop.HOLD, wait=False)
                motorRight.run_angle(GOATED_SPEED, getAngle(BACKTRACK), then=Stop.HOLD, wait=True)
                turnRightish(120)
                hitObject = True
    
    if hitObject:
        facingObjective = False
        TARGET_DISTANCE = 150
        distance = ultrasonic.distance()
        if(distance is None):
            distance = lastDistance
        error = TARGET_DISTANCE - distance
        # Avoid freakouts by clamping error
        # OLD: 70
        # 2nd old: 120
        MAX_TURN = 120
        error = max(-MAX_TURN, min(MAX_TURN, error))
        turn = error * 0.85
        runMotors(GOATED_SPEED + turn, GOATED_SPEED - turn)
        x, y, heading = getUpdatedPosition(motorLeft, motorRight, gyro)
        ev3.screen.clear()
        ev3.screen.draw_text(0, 0,  "x: " + str(round(x)))
        ev3.screen.draw_text(0, 30, "y: " + str(round(y)))
        ev3.screen.draw_text(0, 60, "theta: " + str(round((math.degrees(heading) % 360))))
        lastDistance = distance
        
        if (touchPressed()): # make sure this is counted in the math
            BACKTRACK = -2
            motorLeft.run_angle(GOATED_SPEED, getAngle(BACKTRACK), then=Stop.HOLD, wait=False)
            motorRight.run_angle(GOATED_SPEED, getAngle(BACKTRACK), then=Stop.HOLD, wait=True)
            turnRightish(120)

        if onMLine() and dist_to_goal(x,y) < dist_to_goal(LASTHITX,LASTHITY):
            turnTowardPoint(250, 250)
            facingObjective = True
            hitObject = False

    if (not hitObject and not onMLine() and facingObjective):
        runMotors(GOATED_SPEED, GOATED_SPEED)
        if (touchPressed()):
                LASTHITX = x
                LASTHITY = y
                BACKTRACK = -2
                motorLeft.run_angle(GOATED_SPEED, getAngle(BACKTRACK), then=Stop.HOLD, wait=False)
                motorRight.run_angle(GOATED_SPEED, getAngle(BACKTRACK), then=Stop.HOLD, wait=True)
                turnRightish(120)
                hitObject = True   
    
    if (250 - x) < 25 and (250 - y) < 25:
        # FINISH CODE
        motorLeft.hold()
        motorRight.hold()
        ev3.speaker.say('twenty one')

        

#########################################################################################
    # if (currentTask == "APPROACH"):
    #     if (not taskStarted):
    #         if (centerButtonPressed()):
    #             motorLeft.reset_angle(0)
    #             motorRight.reset_angle(0)
    #             runMotors(GOATED_SPEED, GOATED_SPEED)
    #             taskStarted = True

    #     else:
    #         if (touch.pressed()):
    #             motorLeft.hold()
    #             motorRight.hold()

    #             startDistance = ((motorLeft.angle() +  motorRight.angle()) / 2 / 360) * (2 * math.pi * WHEELRADIUS)
    #             BACKTRACK = -4

    #             motorLeft.run_angle(GOATED_SPEED, getAngle(BACKTRACK), then=Stop.HOLD, wait=False)
    #             motorRight.run_angle(GOATED_SPEED, getAngle(BACKTRACK), then=Stop.HOLD, wait=True)

    #             goNext("CIRCLE")
    # if (currentTask == "CIRCLE"):
    #     # mm 
    #     TARGET_DISTANCE = 150

    #     if (not taskStarted):
    #         # rotate tj and start the task
    #         turnRightish(120)
    #         motorLeft.hold()
    #         motorRight.hold()
    #         motorLeft.reset_angle(0)
    #         motorRight.reset_angle(0)
    #         gyro.reset_angle(0)
    #         lastDistance = 40
    #         prevLeft = motorLeft.angle()
    #         prevRight = motorRight.angle()
    #         prevGyro = gyro.angle()
    #         taskStarted = True    
    #     else:
    #         distance = ultrasonic.distance()
    #         if(distance is None):
    #             distance = lastDistance
    #         error = TARGET_DISTANCE - distance
    #         # Avoid freakouts by clamping error
    #         # OLD: 70
    #         # 2nd old: 120
    #         MAX_TURN = 120
    #         error = max(-MAX_TURN, min(MAX_TURN, error))
    #         turn = error * 0.85
    #         runMotors(GOATED_SPEED + turn, GOATED_SPEED - turn)
    #         x, y, heading = getUpdatedPosition(motorLeft, motorRight, gyro)
    #         ev3.screen.clear()
    #         ev3.screen.draw_text(0, 0,  "x: " + str(round(x)))
    #         ev3.screen.draw_text(0, 30, "y: " + str(round(y)))
    #         ev3.screen.draw_text(0, 60, "theta: " + str(round((math.degrees(heading) % 360))))

    #         if (touch.pressed()):
    #             BACKTRACK = -2
    #             motorLeft.run_angle(GOATED_SPEED, getAngle(BACKTRACK), then=Stop.HOLD, wait=False)
    #             motorRight.run_angle(GOATED_SPEED, getAngle(BACKTRACK), then=Stop.HOLD, wait=True)
    #             turnRightish(120)

    #         if math.sqrt((x * x) + (y * y)) < 5 and totalDistance > 15:
    #             goNext("DONE")
    #         lastDistance = distance
    # if (currentTask == "DONE"):
    #     if (not taskStarted): 
    #         endDist = startDistance - (ultrasonic.distance() / 10) - 20
    #         ev3.screen.clear()
    #         turnRightish(120)
    #         motorLeft.run_angle(GOATED_SPEED, getAngle(endDist), then=Stop.HOLD, wait=False)
    #         motorRight.run_angle(GOATED_SPEED, getAngle(endDist), then=Stop.HOLD, wait=True)
    #         ev3.speaker.beep()
    #         ev3.speaker.set_speech_options('en', 'm1', 150, 99)
    #         ev3.screen.print('9 + 10 ?')
    #         taskStarted = True
    #     ev3.speaker.say('twenty one')
    #     wait(750)
    # wait(5)

    

