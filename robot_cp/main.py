import sys
import board
import busio
import adafruit_pca9685
from time import sleep, monotonic
import board
import pulseio
from digitalio import DigitalInOut, Direction, Pull
from adafruit_motor import motor, servo
from adafruit_servokit_alice import ServoKit
import adafruit_rfm9x
from analogio import AnalogIn
import math
from  random import random
from adafruit_ht16k33.matrix import Matrix8x8
import adafruit_lis3mdl
import struct

def log_error(exc):
    sys.print_exception(exc)

#constants
RADIO_TIMEOUT = 0.5
MIN_ANGLE_CHANGE = 5
SMOOTHING_ANGLE = 3
MAX_SPEED = 1
DELAY_THROTTLE = 1
TURN_SPEED = 1
MIN_SPEED = 0.35
RIGHT_WHEEL_MULTIPLIER = 1 #0.95
LEFT_WHEEL_MULTIPLIER = 1
MAX_WHEEL_DELAY = 0.8
SERVO_INIT_DELAY = 0.25

#gripper_constants
GRIPPER_TIGHTNESS = 60

GRIPPER_TOLERANCE = 5

GRIPPER_MOTION = 10
LEFT_GRIPPER_SERVO = 0
RIGHT_GRIPPER_SERVO = 10
GRIPPER_OPEN_ANGLE = 180
GRIPPER_CLOSE_ANGLE = 60
GRIPPER_CLOSE_EASE = 0.40

#initialize servos for arms and head
# LEFT_CLAW = 0
LEFT_ELBOW_PITCH = 1
LEFT_ELBOW_YAW = 2
LEFT_SHOULDER_PITCH = 3
LEFT_SHOULDER_YAW = 4
LEFT_WRIST = 5
RIGHT_WRIST = 11
RIGHT_ELBOW_PITCH = 12
RIGHT_ELBOW_YAW = 13
RIGHT_SHOULDER_PITCH = 14
RIGHT_SHOULDER_YAW = 15
HEAD_YAW = 8
HEAD_PITCH =7

LEFT_WHEEL = 16
RIGHT_WHEEL = 17

HEADING_START = 18
HEADING_STOP = 19
HEADING_MULTIPLIER = 100

CONFIG_FLAGS = 20



class Gripper:
    def __init__(self, servo, sensor):
        self.servo = servo
        self.sensor = sensor
        self.gripping = False
        self.max_grip_angle = GRIPPER_CLOSE_ANGLE + GRIPPER_TIGHTNESS
        self.servo.angle = GRIPPER_OPEN_ANGLE
##	this function will move the gripper toward the target_angle
##    if the gripper is already gripping then it will have a limit on how tight it can get	
    def move_gripper(self, target_angle):
        if not(self.sensor.value):
            if not(self.gripping):
				#this is new! Let's indicate we're gripping and set the max_grip_angle from this point
                self.gripping = True
                self.max_grip_angle = self.servo.angle - GRIPPER_TIGHTNESS
                if self.max_grip_angle < 0:
                    self.max_grip_angle = 0
        else:
##			set gripping to False in case it was set to true previously
            self.gripping = False
            self.max_grip_angle = GRIPPER_CLOSE_ANGLE
##		set the max angle
        if target_angle < self.max_grip_angle:
            target_angle = self.max_grip_angle
        if target_angle > GRIPPER_OPEN_ANGLE:
            target_angle = GRIPPER_OPEN_ANGLE
        if target_angle < GRIPPER_CLOSE_ANGLE:
            target_angle = GRIPPER_CLOSE_ANGLE

##		now we can move the gripper!
        try:
            if math.fabs(target_angle - self.servo.angle) > GRIPPER_TOLERANCE:
                destination_angle = target_angle
                if target_angle > self.servo.angle + SMOOTHING_ANGLE:#and self.servo.angle + GRIPPER_MOTION < target_angle:
                    destination_angle = self.servo.angle + SMOOTHING_ANGLE
                elif target_angle < self.servo.angle - SMOOTHING_ANGLE: #and self.servo.angle - GRIPPER_MOTION > target_angle:
                    destination_angle = self.servo.angle - SMOOTHING_ANGLE
                self.servo.angle = destination_angle
        except ValueError as e: log_error(e)

class Wheel:
    def __init__(self, motor, speed_multiplier):
        self.speed = 0
        self.delayStartTime = monotonic()
        self.delaySeconds = 0.0
        self.motor = motor
        self.speed_multiplier = speed_multiplier
    def convert_byte_to_throttle(self, signed_control_byte):
        #first calculate the speed value based on a signed_control_byte
        # - subtract 127 for a range of -127 to 127
        # - then calculate she speed as a proportion of MAX_SPEED
        speed = ((signed_control_byte - 127)/127.0) * (MAX_SPEED - MIN_SPEED) #+ MIN_SPEED
        if speed > 0:
            speed += MIN_SPEED
        elif speed < 0:
            speed -= MIN_SPEED
        return speed * self.speed_multiplier
    def set_wheel_speed(self, signed_control_byte):
        self.motor.throttle = self.convert_byte_to_throttle(signed_control_byte)

i2c = board.I2C()
servokit = ServoKit(channels=16)

#initialize LoRa radio
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
radio_nss = DigitalInOut(board.A5)
radio_reset = DigitalInOut(board.A4)
radio = adafruit_rfm9x.RFM9x(spi, radio_nss, radio_reset, 915.0)

##initialize left gripper
left_gripper_sensor = DigitalInOut(board.D12)
left_gripper_sensor.direction = Direction.INPUT
left_gripper_sensor.pull = Pull.DOWN
left_gripper = Gripper(servokit.servo[LEFT_GRIPPER_SERVO],left_gripper_sensor)

##initialize right gripper
right_gripper_sensor = DigitalInOut(board.D13)
right_gripper_sensor.direction = Direction.INPUT
right_gripper_sensor.pull = Pull.DOWN
right_gripper = Gripper(servokit.servo[RIGHT_GRIPPER_SERVO],right_gripper_sensor)

##initialize wheel motors
ain1 = pulseio.PWMOut(board.D5)
ain2 = pulseio.PWMOut(board.D6)
wheels_sleep = DigitalInOut(board.D9)
bin2 = pulseio.PWMOut(board.D10)
bin1 = pulseio.PWMOut(board.D11)

wheels_sleep.direction = Direction.OUTPUT
wheels_sleep.value = True
left_wheel_motor = motor.DCMotor(ain1,ain2)
right_wheel_motor = motor.DCMotor(bin1,bin2)
left_wheel_motor.throttle = 0
right_wheel_motor.throttle = 0
left_wheel = Wheel(left_wheel_motor, LEFT_WHEEL_MULTIPLIER)
right_wheel = Wheel(right_wheel_motor, RIGHT_WHEEL_MULTIPLIER)

##battery management
vbat_voltage = AnalogIn(board.VOLTAGE_MONITOR)
 	
def get_voltage(pin):
    return (pin.value * 3.3) / 65536 * 2

#functions

def move_servo(index,command_bytes,index_offset = 0, smoothing=False, delay=False):
    servo = servokit.servo[index]
    new_angle = command_bytes[index + index_offset]
    if abs(servo.angle - new_angle) > MIN_ANGLE_CHANGE:
        try:
            if (smoothing):
                if new_angle > servo.angle:
                    servo.angle+=SMOOTHING_ANGLE
                elif servo.angle - SMOOTHING_ANGLE > 0:
                    servo.angle-=SMOOTHING_ANGLE
            else:
                servo.angle = new_angle

        except Error as e: log_error(e)
    if delay:
        servokit.setServos()
        sleep(SERVO_INIT_DELAY)

#initilize BNO085
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

#i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c)

bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
class EulerOrientation:
    def __init__(self, roll=0,pitch=0,yaw=0):
        self.roll, self.pitch, self.yaw = roll,pitch,yaw

def quaternionToEuler(qw,qx,qy,qz):
    roll  = math.atan2(2.0 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
    pitch = -math.asin(2.0 * (qx * qz - qw * qy))
    yaw   = math.atan2(2.0 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz)
    pitch *= 180.0 / math.pi
    yaw   *= 180.0 / math.pi
    if yaw < 0: yaw   += 360.0  # Ensure yaw stays between 0 and 360
    roll  *= 180.0 / math.pi
    return EulerOrientation(roll=roll,pitch=pitch,yaw=yaw)

def closestAngleToTarget(currentAngle,targetAngle):
    angleDifference = targetAngle - currentAngle
    if math.fabs(angleDifference) <= 180:
        return angleDifference
    else:
        #the angle difference is greater than 180 so we want the inverse angle
        absInverseAngle = math.fabs(angleDifference)-180
        if angleDifference > 0:
            #if original angle differece is counter-clockwise, then we want to go clockwise
            return -1 * absInverseAngle
        else:
            #if original angle differece is clockwise, then we want to go coutner
            return absInverseAngle


def set_orientation(command_bytes, smoothing = False, delay = False, correctHeading = False):
    #move arms and head
    move_servo(HEAD_PITCH, command_bytes, INDEX_OFFSET, smoothing,delay)
    move_servo(HEAD_YAW, command_bytes, INDEX_OFFSET, smoothing,delay)
    move_servo(LEFT_ELBOW_PITCH, command_bytes, INDEX_OFFSET, smoothing,delay)
    move_servo(LEFT_ELBOW_YAW, command_bytes, INDEX_OFFSET, smoothing,delay)
    move_servo(LEFT_SHOULDER_PITCH, command_bytes, INDEX_OFFSET, smoothing,delay)
    move_servo(LEFT_SHOULDER_YAW, command_bytes, INDEX_OFFSET, smoothing,delay)
    move_servo(LEFT_WRIST, command_bytes, INDEX_OFFSET, smoothing,delay)
    move_servo(RIGHT_ELBOW_PITCH, command_bytes, INDEX_OFFSET, smoothing,delay)
    move_servo(RIGHT_ELBOW_YAW, command_bytes, INDEX_OFFSET, smoothing,delay)
    move_servo(RIGHT_SHOULDER_PITCH, command_bytes, INDEX_OFFSET, smoothing,delay)
    move_servo(RIGHT_SHOULDER_YAW, command_bytes, INDEX_OFFSET,smoothing,delay)
    move_servo(RIGHT_WRIST, command_bytes, INDEX_OFFSET,smoothing,delay)


    #correct heading
    qx, qy, qz, qw = bno.quaternion  # pylint:disable=no-member
    eulerOrientation = quaternionToEuler(qw=qw,qx=qx,qy=qy,qz=qz)
    #test angle check
    targetRotation = struct.unpack_from('>H',command_bytes, HEADING_START + INDEX_OFFSET)[0] / HEADING_MULTIPLIER
    precisionTolerance = 3
    currentRotation = eulerOrientation.yaw
    angleToTarget = closestAngleToTarget(currentRotation, targetRotation)
    if correctHeading:
        if math.fabs(angleToTarget) > 45:
            rotationWheelSpeed = TURN_SPEED
        else:
            rotationWheelSpeed = (math.fabs(angleToTarget)/45 * (TURN_SPEED-MIN_SPEED)) + MIN_SPEED
        if angleToTarget >= 0 + precisionTolerance:
            #print("Turn Left\t\tCurrent Rotation:" + str(int(currentRotation)))
            left_wheel.motor.throttle = (
                left_wheel.convert_byte_to_throttle(command_bytes[LEFT_WHEEL + INDEX_OFFSET]) + ( -1 * rotationWheelSpeed)
                if left_wheel.convert_byte_to_throttle(command_bytes[LEFT_WHEEL + INDEX_OFFSET]) + ( -1 * rotationWheelSpeed) >= -1 * MAX_SPEED
                else -1 * MIN_SPEED
            )
            right_wheel.motor.throttle = (
                right_wheel.convert_byte_to_throttle(command_bytes[RIGHT_WHEEL + INDEX_OFFSET]) + rotationWheelSpeed
                if right_wheel.convert_byte_to_throttle(command_bytes[RIGHT_WHEEL + INDEX_OFFSET]) + rotationWheelSpeed <= MAX_SPEED
                else MAX_SPEED
            )
        elif angleToTarget <= 0 - precisionTolerance:
            #print("Turn Right\t\tCurrent Rotation:" + str(int(currentRotation)))
            right_wheel.motor.throttle = (
                right_wheel.convert_byte_to_throttle(command_bytes[RIGHT_WHEEL + INDEX_OFFSET]) + ( -1 * rotationWheelSpeed)
                if right_wheel.convert_byte_to_throttle(command_bytes[RIGHT_WHEEL + INDEX_OFFSET]) + ( -1 * rotationWheelSpeed) >= -1 * MAX_SPEED
                else -1 * MIN_SPEED
            )
            left_wheel.motor.throttle = (
                left_wheel.convert_byte_to_throttle(command_bytes[LEFT_WHEEL + INDEX_OFFSET]) + rotationWheelSpeed
                if left_wheel.convert_byte_to_throttle(command_bytes[LEFT_WHEEL + INDEX_OFFSET]) + rotationWheelSpeed <= MAX_SPEED
                else MAX_SPEED
            )
        else:
            #print("Target Reached!\t\tCurrent Rotation:" + str(int(currentRotation)))
            left_wheel.set_wheel_speed(command_bytes[LEFT_WHEEL+ INDEX_OFFSET])
            right_wheel.set_wheel_speed(command_bytes[RIGHT_WHEEL + INDEX_OFFSET])
    else:
        left_wheel.set_wheel_speed(command_bytes[LEFT_WHEEL+ INDEX_OFFSET])
        right_wheel.set_wheel_speed(command_bytes[RIGHT_WHEEL + INDEX_OFFSET])
    #set wheel speed



    #left_wheel.set_wheel_speed(command_bytes[LEFT_WHEEL+ INDEX_OFFSET])
    #right_wheel.set_wheel_speed(command_bytes[RIGHT_WHEEL + INDEX_OFFSET])
    #set gripper
    # if delay:
    #move_servo(LEFT_GRIPPER_SERVO, command_bytes, INDEX_OFFSET,smoothing,delay)
    #move_servo(RIGHT_GRIPPER_SERVO, command_bytes, INDEX_OFFSET,smoothing,delay)
#     else:
    left_gripper.move_gripper(command_bytes[LEFT_GRIPPER_SERVO + INDEX_OFFSET])
    right_gripper.move_gripper(command_bytes[RIGHT_GRIPPER_SERVO + INDEX_OFFSET])
	
    servokit.setServos()


#BEGIN EYES
#eye direction constants
LOOK_CENTER = 0
LOOK_UP = 1
LOOK_DOWN = -1
LOOK_RIGHT = 1
LOOK_LEFT = -1

EYE_BLINK_DURATION = 0.08
EYE_BLINK_MIN_DELAY = 2
EYE_BLINK_CHANCE = 0.5

right_eye = Matrix8x8(i2c, auto_write=False)
right_eye.brightness = 1
right_eye.fill(1)
left_eye = Matrix8x8(i2c, address=0x71, auto_write=False)
left_eye.brightness = 1
right_eye.fill(1)
def setMatrixframe(right_eye, left_eye, right_frame, left_frame):
    right_eye.fill(1)
    for y in range(len(right_frame)):
        for x in range(len(right_frame[y])):
            right_eye[y,x] = right_frame[x][y]
    left_eye.fill(1)
    for y in range(len(left_frame)):
        for x in range(len(left_frame[y])):
            left_eye[y,x] = left_frame[x][y]

def setEyeFrameByDirection(eye_direction):
    global current_left_frame, current_right_frame, update_eyes

    new_left_frame = current_left_frame
    new_right_frame = current_right_frame

    if eye_direction == [LOOK_CENTER, LOOK_CENTER]:
        new_left_frame = frame_left_eye_forward
        new_right_frame = frame_right_eye_forward
    if eye_direction == [LOOK_LEFT, LOOK_CENTER]:
        new_left_frame = frame_look_left
        new_right_frame = frame_look_left
    if eye_direction == [LOOK_RIGHT, LOOK_CENTER]:
        new_left_frame = frame_look_right
        new_right_frame = frame_look_right
    if eye_direction == [LOOK_CENTER, LOOK_UP]:
        new_left_frame = frame_left_eye_up
        new_right_frame = frame_right_eye_up
    if eye_direction == [LOOK_LEFT, LOOK_UP]:
        new_left_frame = frame_look_up_left
        new_right_frame = frame_look_up_left
    if eye_direction == [LOOK_RIGHT, LOOK_UP]:
        new_left_frame = frame_look_up_right
        new_right_frame = frame_look_up_right
    if eye_direction == [LOOK_CENTER, LOOK_DOWN]:
        new_left_frame = frame_left_eye_down
        new_right_frame = frame_right_eye_down
    if eye_direction == [LOOK_LEFT, LOOK_DOWN]:
        new_left_frame = frame_look_down_left
        new_right_frame = frame_look_down_left
    if eye_direction == [LOOK_RIGHT, LOOK_DOWN]:
        new_left_frame = frame_look_down_right
        new_right_frame = frame_look_down_right
    if new_left_frame != current_left_frame or new_right_frame != current_right_frame:
        current_left_frame = new_left_frame
        current_right_frame = new_right_frame
        update_eyes = True

def getEyeDirectionByOrientation():
    eye_x = LOOK_CENTER
    eye_y = LOOK_CENTER
    x_tolerance = 10
    y_tolerance = 10

    #first set eye_x based on whether head is currently moving left or right
    if command_bytes[HEAD_YAW + INDEX_OFFSET] < servokit.servo[HEAD_YAW].angle - x_tolerance:
        eye_x = LOOK_LEFT
    elif command_bytes[HEAD_YAW + INDEX_OFFSET] > servokit.servo[HEAD_YAW].angle + x_tolerance:
        eye_x = LOOK_RIGHT

    #next we set eye_y whether the angle is up center or down. Unlike x, y stays in place even after reached.
    if command_bytes[HEAD_PITCH + INDEX_OFFSET] > 90 + y_tolerance:
        eye_y = LOOK_UP
    if command_bytes[HEAD_PITCH + INDEX_OFFSET] < 90 - y_tolerance:
        eye_y = LOOK_DOWN


    return [eye_x, eye_y]



frame_right_eye_forward = [
    [0,1,1,1,1,1,1,0],
    [1,1,1,1,1,1,1,1],
    [1,1,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [0,1,0,0,0,0,0,0]
]
frame_left_eye_forward = [
    [0,1,1,1,1,1,1,0],
    [1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,1,0]
]
frame_right_eye_up = [
    [0,1,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1],
    [0,1,1,1,1,1,1,0]
]
frame_left_eye_up = [
    [0,0,0,0,0,0,1,0],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,1,1],
    [1,1,1,1,1,1,1,1],
    [0,1,1,1,1,1,1,0],
]
frame_right_eye_down = [
    [0,1,1,1,1,1,1,0],
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
    [1,1,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0],
]
frame_left_eye_down = [
    [0,1,1,1,1,1,1,0],
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,0],
]

frame_look_right = [
    [0,1,1,1,1,1,1,0],
    [1,1,1,1,1,1,1,1],
    [1,1,1,0,0,0,0,1],
    [1,1,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0],
    [0,1,1,0,0,0,0,0]
]

frame_look_left = [
    [0,1,1,1,1,1,1,0],
    [1,1,1,1,1,1,1,1],
    [1,0,0,0,0,1,1,1],
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,1,1,0]
]

frame_look_up_right = [
    [0,1,1,0,0,0,0,0],
    [1,1,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0],
    [1,1,1,0,0,0,0,0],
    [1,1,1,1,1,1,1,1],
    [0,1,1,1,1,1,1,0]
]

frame_look_up_left = [
    [0,0,0,0,0,1,1,0],
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,1,1,1],
    [1,1,1,1,1,1,1,1],
    [0,1,1,1,1,1,1,0]
]
frame_look_down_right = [
    [0,1,1,1,1,1,1,0],
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
    [1,1,1,0,0,0,0,0],
    [1,1,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0],
    [0,1,0,0,0,0,0,0],
]
frame_look_down_left = [
    [0,1,1,1,1,1,1,0],
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
    [0,0,0,0,0,1,1,1],
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,1,0],
]


frame_blink = [
    [0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0],
    [0,1,1,1,1,1,1,0],
    [1,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0]
]

command_bytes = [90] * 64
command_bytes[LEFT_GRIPPER_SERVO] = GRIPPER_CLOSE_ANGLE
command_bytes[RIGHT_GRIPPER_SERVO] = GRIPPER_OPEN_ANGLE
command_bytes[LEFT_ELBOW_PITCH] = 90
command_bytes[LEFT_ELBOW_YAW] = 90
command_bytes[LEFT_SHOULDER_PITCH] = 180
command_bytes[LEFT_SHOULDER_YAW] = 135
command_bytes[HEAD_PITCH] =65
command_bytes[HEAD_YAW] = 90
command_bytes[RIGHT_WRIST] = 90
command_bytes[RIGHT_ELBOW_PITCH] = 120
command_bytes[RIGHT_ELBOW_YAW] = 90
command_bytes[RIGHT_SHOULDER_PITCH] = 0
command_bytes[RIGHT_SHOULDER_YAW] = 45
command_bytes[LEFT_WHEEL] =  0 + 127
command_bytes[RIGHT_WHEEL] = 0 + 127
command_bytes[LEFT_WRIST] = 90

INDEX_OFFSET = 0

update_eyes = False
current_right_frame = None
current_left_frame = None
eye_direction=[LOOK_CENTER, LOOK_CENTER]
setEyeFrameByDirection(eye_direction)
current_left_frame = frame_blink
current_right_frame = frame_blink
setMatrixframe(right_eye, left_eye, current_right_frame, current_left_frame)
right_eye.show()
left_eye.show()
#main loop
PACKET_SIGNATURE = "ALICE"
INDEX_OFFSET = len(PACKET_SIGNATURE)
command_bytes = bytearray(PACKET_SIGNATURE) + bytearray(command_bytes)
set_orientation(command_bytes=command_bytes, delay=False)

#WAKE UP!
right_eye.show()
left_eye.show()
sleep(2)
setMatrixframe(right_eye, left_eye, frame_right_eye_forward, frame_left_eye_forward)
right_eye.show()
left_eye.show()
sleep(0.25)
setMatrixframe(right_eye, left_eye, frame_blink, frame_blink)
right_eye.show()
left_eye.show()
sleep(EYE_BLINK_DURATION)
setMatrixframe(right_eye, left_eye, frame_right_eye_forward, frame_left_eye_forward)
right_eye.show()
left_eye.show()
sleep(0.10)
setMatrixframe(right_eye, left_eye, frame_blink, frame_blink)
right_eye.show()
left_eye.show()
sleep(EYE_BLINK_DURATION)
current_left_frame = frame_right_eye_forward
current_right_frame = frame_left_eye_forward
setMatrixframe(right_eye, left_eye, frame_right_eye_forward, frame_left_eye_forward)
right_eye.show()
left_eye.show()
command_bytes[HEAD_PITCH + INDEX_OFFSET] = 90
eye_blink_timer = monotonic()
radio_timer = monotonic()
eyes_blinking = False
lastHeadMovement = monotonic()



demo=True
correctHeading = False
while True:
    loopStart = monotonic()

    if monotonic() - radio_timer > RADIO_TIMEOUT and demo:
        correctHeading = False
        left_wheel.motor.throttle = 0
        right_wheel.motor.throttle = 0
        if (lastHeadMovement == None or loopStart-lastHeadMovement > 1):
            if random() > 0.5:
                if random() > 0.3:
                    command_bytes[HEAD_PITCH + INDEX_OFFSET] = int(random()*90) + 45
                    command_bytes[HEAD_YAW + INDEX_OFFSET] = int(random()*90) + 45
                if random() > 0.3:
                    command_bytes[LEFT_GRIPPER_SERVO + INDEX_OFFSET] = GRIPPER_OPEN_ANGLE + int(random()*(GRIPPER_CLOSE_ANGLE - GRIPPER_OPEN_ANGLE))
                if random() > 0.3:
                    command_bytes[RIGHT_GRIPPER_SERVO + INDEX_OFFSET] = GRIPPER_OPEN_ANGLE + int(random()*(GRIPPER_CLOSE_ANGLE - GRIPPER_OPEN_ANGLE))
                if random() > 0.67:
                    command_bytes[RIGHT_ELBOW_PITCH + INDEX_OFFSET] = 135-int(random()*90)
                    command_bytes[RIGHT_ELBOW_YAW + INDEX_OFFSET] = 135-int(random()*90)
                    command_bytes[RIGHT_SHOULDER_PITCH + INDEX_OFFSET] = int(random()*91)
                    command_bytes[RIGHT_SHOULDER_YAW + INDEX_OFFSET] = 45 + int(random()*45)
                    command_bytes[RIGHT_WRIST + INDEX_OFFSET] = int(random()*181)
                if random() > 0.67:
                    command_bytes[LEFT_SHOULDER_YAW + INDEX_OFFSET] = 135 - int(random()*45)
                    command_bytes[LEFT_SHOULDER_PITCH + INDEX_OFFSET] = int(random()*91)+90
                    command_bytes[LEFT_ELBOW_PITCH + INDEX_OFFSET] = 135 - int(random()*90)
                    command_bytes[LEFT_ELBOW_YAW + INDEX_OFFSET] = 45 + int(random()*90)
                    command_bytes[LEFT_WRIST + INDEX_OFFSET] = int(random()*181)
            lastHeadMovement = loopStart
    packet = None
    packet = radio.receive(timeout=0)
    try:
        if packet !=None and len(packet) > 0:
            signature_check = ''
            for i in range(INDEX_OFFSET):
                signature_check+=chr(packet[i])
            if signature_check == PACKET_SIGNATURE:
                radio_timer = monotonic()
                command_bytes = packet
                correctHeading = True
    except Exception as e:
        log_error(e)

    try:
        set_orientation(command_bytes=command_bytes,smoothing=True, correctHeading=correctHeading)
    except Exception as e:
        log_error(e)
    setEyeFrameByDirection(getEyeDirectionByOrientation())
    #blinking
    if not eyes_blinking:
        if monotonic() - eye_blink_timer > EYE_BLINK_MIN_DELAY:
            if random() < EYE_BLINK_CHANCE:
                previous_left_frame = current_left_frame
                previous_right_frame = current_right_frame
                current_right_frame = frame_blink
                current_left_frame = frame_blink
                eye_blink_timer = monotonic()
                eyes_blinking = True
                update_eyes = True
            else:
                #restart blink timer
                eye_blink_timer = monotonic()
    elif eyes_blinking and (monotonic() - eye_blink_timer) > EYE_BLINK_DURATION:
        current_right_frame = previous_right_frame
        current_left_frame = previous_left_frame
        eye_blink_timer = monotonic()
        eyes_blinking = False
        update_eyes = True
    else:
        update_eyes = False

    if update_eyes:
        setMatrixframe(right_eye, left_eye, current_right_frame, current_left_frame)
        right_eye.show()
        left_eye.show()
        update_eyes = False



    # print("R: %0.6f  P: %0.6f Y: %0.6f" % (eulerOrientation.roll, eulerOrientation.pitch, eulerOrientation.yaw))
#     print("")

    loopLength = monotonic() - loopStart
    fixedLoop = 0.04
    if loopLength < fixedLoop:
        sleep(fixedLoop - loopLength)