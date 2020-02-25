import board
import busio
import adafruit_pca9685
import time
import board
import pulseio
from adafruit_motor import motor
from adafruit_servokit import ServoKit

#initialize servos for arms and head
LEFT_CLAW = 0
LEFT_ELBOW_PITCH = 1
LEFT_ELBOW_YAW = 2
LEFT_SHOULDER_PITCH = 3
LEFT_SHOULDER_YAW = 4
RIGHT_CLAW = 11
RIGHT_ELBOW_PITCH = 12
RIGHT_ELBOW_YAW = 13
RIGHT_SHOULDER_PITCH = 14
RIGHT_SHOULDER_YAW = 15
HEAD_YAW = 8
HEAD_PITCH =7
servokit = ServoKit(channels=16)

#initialize wheel motors
ain1 = pulseio.PWMOut(board.D13)
ain2 = pulseio.PWMOut(board.D12)
bin1 = pulseio.PWMOut(board.D11)
#D10 fails with "all timers for this pin are in use" Something else must be using it?
bin2 = pulseio.PWMOut(board.D6)
left_wheel = motor.DCMotor(ain1,ain2)
right_wheel = motor.DCMotor(bin1,bin2)