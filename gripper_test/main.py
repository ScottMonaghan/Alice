import board
import busio
import adafruit_pca9685
from time import sleep
import board
import pulseio
from digitalio import DigitalInOut, Direction, Pull
from adafruit_motor import motor
from adafruit_servokit_alice import ServoKit
import adafruit_rfm9x
from analogio import AnalogIn
import math
 
#constants
EASE_IN = 0.1
GRIP_TIGHTNESS = 30
TOLERANCE = 3
MOTION = 3

#init servokit
servokit = ServoKit(channels=16)
claw = servokit.servo[0]

grip_sensor = DigitalInOut(board.A0)
grip_sensor.direction = Direction.INPUT
grip_sensor.pull = Pull.DOWN
#claw.angle = 0

target_angle=0

while True:
    if not(grip_sensor.value) and (target_angle > claw.angle + GRIP_TIGHTNESS):
        target_angle = claw.angle + GRIP_TIGHTNESS
        
    if math.fabs(claw.angle - target_angle)>TOLERANCE:
        if target_angle > (claw.angle+TOLERANCE): claw.angle += MOTION
        elif target_angle < claw.angle: claw.angle -= MOTION
    elif claw.angle > 0:
        target_angle = claw.angle
        claw.angle = target_angle
    print(str(grip_sensor.value)+ " " + str(target_angle) + " " + str(claw.angle))
    #sleep(0.05)
    