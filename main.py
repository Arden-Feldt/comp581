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
WHEELRADIUS = 2.8
GOATED_SPEED = 300
WHEEL_BASE = 12
START_THETA = math.radians(90)
START_X = 50 + 16
START_Y = -30
GOAL_X = 250
GOAL_Y = 250

# Vars
# Dead reckoning
x = START_X
y = START_Y
theta = START_THETA
totalDistance = 0
prevLeft = 0
prevRight = 0
prevGyro = START_THETA
hitObject = False
facingObjective = False

# Algorithm
lastHitX = 0
lastHitY = 0
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

def centerButtonPressed():
    pressed = (Button.CENTER in ev3.buttons.pressed())
    if (pressed):
        wait(100)
    return pressed

def runMotors(speedLeft, speedRight):
    motorLeft.run(speedLeft)
    motorRight.run(speedRight)

def turnByDegrees(angleDeg, speed=100):
    updatePose()
    startAngle = math.degrees(theta)
    targetAngle = startAngle + angleDeg

    while True:
        updatePose()
        currentAngle = math.degrees(theta)
        error = (targetAngle - currentAngle + 180) % 360 - 180
        if abs(error) < 1:
            break

        if (touchPressed()):
            break
        if error > 0:
            motorLeft.run(-speed)
            motorRight.run(speed)
        else:
            motorLeft.run(speed)
            motorRight.run(-speed)

        wait(20)

    motorLeft.hold()
    motorRight.hold()
    wait(100)


def backtrack(distance = 2.5):
    wait(100)
    motorLeft.run_angle(GOATED_SPEED, getAngle(-distance), then=Stop.HOLD, wait=False)
    motorRight.run_angle(GOATED_SPEED, getAngle(-distance), then=Stop.HOLD, wait=True)
    updatePose()

theta_d = 0
def updatePose():
    global x, y, theta, totalDistance, prevLeft, prevRight, prevGyro

    angleLeft = motorLeft.angle()
    angleRight = motorRight.angle()
    angleGyro = -math.radians(gyro.angle())

    # Wheel distances
    deltaLeft = math.radians(angleLeft - prevLeft) * (WHEELRADIUS)
    deltaRight = math.radians(angleRight - prevRight) * (WHEELRADIUS)
    deltaAvg = (deltaLeft + deltaRight) / 2
    totalDistance += deltaAvg

    # Heading
    deltaTheta = angleGyro - prevGyro
    
    # theta, x, y
    theta += deltaTheta
    x += deltaAvg * math.cos(theta)
    y += deltaAvg * math.sin(theta)
    
    # Save for next loop
    prevLeft = angleLeft
    prevRight = angleRight
    prevGyro = angleGyro

    ev3.screen.clear()
    ev3.screen.draw_text(0, 0,  "x: " + str(round(x)))
    ev3.screen.draw_text(0, 30, "y: " + str(round(y)))
    ev3.screen.draw_text(0, 60, "theta: " + str(round((math.degrees(theta) % 360))))
    ev3.screen.draw_text(0, 100, "theta_d " + str(theta_d))
    # ev3.screen.draw_text(0, 130, "facingObjective: " + str(facingObjective))
    # ev3.screen.draw_text(0, 160, "hitObject: " + str(hitObject))

def onMLine():
    global x, y
    m = (GOAL_Y - START_Y) / (GOAL_X - START_X)
    b = START_Y - m * START_X

    if abs((m * x + b) - y) < 10:
        return True
    return False

def distToGoal(x, y, goalX=GOAL_X, goalY=GOAL_Y):
    dx = goalX - x
    dy = goalY - y
    return math.sqrt(dx*dx + dy*dy)

def turnTowardPoint(targetX, targetY):
    global x, y, theta, theta_d

    desiredTheta = math.atan2(targetY - y, targetX - x)
    angleError = desiredTheta - theta

    angleError = (angleError + math.pi) % (2 * math.pi) - math.pi

    angleDeg = math.degrees(angleError)

    theta_d = math.degrees(desiredTheta)
    turnByDegrees(angleDeg)
    updatePose()

# def facingObjectiveish(targetX, targetY):
#     desiredTheta = math.atan2(targetY - y, targetX - x)
#     angleError = desiredTheta - theta

#     angleError = (angleError + math.pi) % (2 * math.pi) - math.pi

#     return abs(math.degrees(angleError)) < 2

# Algorithm
lastHitX = 0
lastHitY = 0
lastDistance = 0
def main():
    global x, y, theta, totalDistance, lastHitX, lastHitY, lastDistance, hitObject, facingObjective
    gyro.reset_angle(math.degrees(-START_THETA))
    motorLeft.reset_angle(0)
    motorRight.reset_angle(0)

    while (True):
        updatePose()
        # wait(20)
        # continue
        if (not hitObject):
            if (touchPressed()):
                lastHitX = x
                lastHitY = y
                backtrack()
                turnByDegrees(-90)
                hitObject = True
                continue

            if (not facingObjective):
                turnTowardPoint(GOAL_X, GOAL_Y)
                facingObjective = True

            if (onMLine() and facingObjective):
                runMotors(GOATED_SPEED, GOATED_SPEED)

                # facingObjective = onMLine() or facingObjectiveish(250, 250)
        else:
            facingObjective = False
            TARGET_DISTANCE = 150

            # Handle errors
            distance = ultrasonic.distance()
            if(distance is None):
                distance = lastDistance
            else:
                lastDistance = distance

            error = TARGET_DISTANCE - distance
            # Avoid freakouts by clamping error
            # OLD: 70
            MAX_TURN = 120
            error = max(-MAX_TURN, min(MAX_TURN, error))
            turn = error * 0.85
            runMotors(GOATED_SPEED + turn, GOATED_SPEED - turn)
            
            # Failsafe
            if (touchPressed()):
                motorLeft.hold()
                motorRight.hold()
                backtrack(7)
                turnByDegrees(-90)
#  
            if onMLine() and distToGoal(x,y) < distToGoal(lastHitX, lastHitY):
                turnTowardPoint(GOAL_X, GOAL_Y)
                facingObjective = True
                hitObject = False

        if (abs(GOAL_X - x) < 10) and (abs(GOAL_Y - y) < 10):
            motorLeft.hold()
            motorRight.hold()
            ev3.speaker.say('twenty one')
        wait(20)
main()