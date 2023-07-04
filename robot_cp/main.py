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

#constants
MIN_ANGLE_CHANGE = 5
SMOOTHING_ANGLE = 3
MAX_SPEED = 1
DELAY_THROTTLE = 0.8
MIN_SPEED = 0.4
MAX_WHEEL_DELAY = 0.2
DEMO_SMOOTHING_FRACTION = 0.1
MOCAP_SMOOTHING_FRACTION = 0.4

#gripper_constants
GRIPPER_TIGHTNESS = 30
GRIPPER_TOLERANCE = 5
GRIPPER_MOTION = 10
LEFT_GRIPPER_SERVO = 0
RIGHT_GRIPPER_SERVO = 10
GRIPPER_OPEN_ANGLE = 90
GRIPPER_CLOSE_ANGLE = 180
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
                self.max_grip_angle = self.servo.angle + GRIPPER_TIGHTNESS
        else:
##			set gripping to False in case it was set to true previously
            self.gripping = False
            self.max_grip_angle = GRIPPER_CLOSE_ANGLE
##		set the max angle
        if target_angle > self.max_grip_angle:
            target_angle = self.max_grip_angle
        if target_angle < GRIPPER_OPEN_ANGLE:
            target_angle = GRIPPER_OPEN_ANGLE
        if target_angle > GRIPPER_CLOSE_ANGLE:
            target_angle = GRIPPER_CLOSE_ANGLE

##		now we can move the gripper!
        try:
            if math.fabs(target_angle - self.servo.angle) > GRIPPER_TOLERANCE:
                destination_angle = target_angle
                if target_angle > self.servo.angle and self.servo.angle + GRIPPER_MOTION < target_angle:
                    destination_angle = self.servo.angle + GRIPPER_CLOSE_EASE * (target_angle-self.servo.angle) #self.servo.angle + GRIPPER_MOTION
                elif target_angle < self.servo.angle and self.servo.angle - GRIPPER_MOTION > target_angle:
                    destination_angle = self.servo.angle - GRIPPER_MOTION
                print(str(target_angle) + " " + str(destination_angle) + " " + str(self.servo.angle) + " " + str(self.gripping) + " " + str(self.sensor.value))
                self.servo.angle = destination_angle
        except ValueError as e: print(e)

class Wheel:
    def __init__(self, motor):
        self.speed = 0
        self.delayStartTime = monotonic()
        self.delaySeconds = 0.0
        self.motor = motor
    def set_wheel_speed(self, signed_control_byte):
        #first calculate the speed value based on a signed_control_byte
        # - subtract 127 for a range of -127 to 127
        # - then calculate she speed as a proportion of MAX_SPEED
        speed = ((signed_control_byte - 127)/127.0) * MAX_SPEED
        #here's where it gets interesting: if we try to go lower than MIN_SPEED
        #then the motors stall, so we're going to pulse max speed and then introduce a delay
        if math.fabs(speed) < MIN_SPEED and math.fabs(speed) > 0:
            #first we want to scale the delay based on the current speed
            #where speed 0 = MAX_WHEEL_DELAY * 1, and MIN_SPEED = MAX_WHEEL_DELAY * 0
            delay = MAX_WHEEL_DELAY * ((MIN_SPEED - math.fabs(speed))/float(MIN_SPEED))
            self.delaySeconds = monotonic() - self.delayStartTime
            if self.delaySeconds >= delay:
               #we're past the delay, send a pulse at MIN_SPEED and start the delay clock again
                if speed > 0:
                    self.motor.throttle = DELAY_THROTTLE
                else:
                    self.motor.throttle = DELAY_THROTTLE * -1
                self.delayStartTime = monotonic()
            else:
                self.motor.throttle = 0
                #self.delayLoops+=1
        else:
            self.motor.throttle = speed

servokit = ServoKit(channels=16)

#initialize LoRa radio
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
radio_nss = DigitalInOut(board.A5)
radio_reset = DigitalInOut(board.A4)
radio = adafruit_rfm9x.RFM9x(spi, radio_nss, radio_reset, 915.0)

##initialize gripper
left_gripper_sensor = DigitalInOut(board.D12)
left_gripper_sensor.direction = Direction.INPUT
left_gripper_sensor.pull = Pull.DOWN
left_gripper = Gripper(servokit.servo[LEFT_GRIPPER_SERVO],left_gripper_sensor)

##initialize wheel motors
ain1 = pulseio.PWMOut(board.D5)
ain2 = pulseio.PWMOut(board.D6)
wheels_sleep = DigitalInOut(board.D9)
bin2 = pulseio.PWMOut(board.D10)
bin1 = pulseio.PWMOut(board.D11)

wheels_sleep.direction = Direction.OUTPUT
wheels_sleep.value = True
left_wheel_motor = motor.DCMotor(bin2,bin1)
right_wheel_motor = motor.DCMotor(ain2,ain1)
left_wheel_motor.throttle = 0
right_wheel_motor.throttle = 0
left_wheel = Wheel(left_wheel_motor)
right_wheel = Wheel(right_wheel_motor)

##battery management
vbat_voltage = AnalogIn(board.VOLTAGE_MONITOR)
 	
def get_voltage(pin):
    return (pin.value * 3.3) / 65536 * 2

#functions

smoothing_fraction = DEMO_SMOOTHING_FRACTION
def move_servo(index,command_bytes,index_offset = 0, smoothing=False):
   servo = servokit.servo[index]
   new_angle = command_bytes[index + index_offset]
   if abs(servo.angle - new_angle) > MIN_ANGLE_CHANGE:
        try:
            if (smoothing):
                # if new_angle > servo.angle:
                #     servo.angle+=SMOOTHING_ANGLE
                # elif servo.angle - SMOOTHING_ANGLE > 0:
                #     servo.angle-=SMOOTHING_ANGLE

                servo.angle += (new_angle - servo.angle) * smoothing_fraction
#                 smooth_angle = (new_angle - servo.angle) * 0.5
#                 if (smooth_angle > SMOOTHING_ANGLE):
#                     smooth_angle = SMOOTHING_ANGLE
#                 if (smooth_angle < -1*SMOOTHING_ANGLE):
#                     smooth_angle = -1*SMOOTHING_ANGLE
#                 servo.angle += smooth_angle
            else:
                servo.angle = new_angle

        except Exception as e: print(e)

def set_orientation(command_bytes, smoothing = False):
    #move arms and head
    # move_servo(LEFT_CLAW, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(LEFT_ELBOW_PITCH, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(LEFT_ELBOW_YAW, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(LEFT_SHOULDER_PITCH, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(LEFT_SHOULDER_YAW, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(LEFT_WRIST, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(HEAD_PITCH, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(HEAD_YAW, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(RIGHT_WRIST, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(RIGHT_ELBOW_PITCH, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(RIGHT_ELBOW_YAW, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(RIGHT_SHOULDER_PITCH, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(RIGHT_SHOULDER_YAW, command_bytes, INDEX_OFFSET,smoothing)
    move_servo(RIGHT_WRIST, command_bytes, INDEX_OFFSET,smoothing)
    move_servo(LEFT_GRIPPER_SERVO, command_bytes, INDEX_OFFSET,smoothing)
    move_servo(RIGHT_GRIPPER_SERVO, command_bytes, INDEX_OFFSET,smoothing)

    #set wheel speed
    left_wheel.set_wheel_speed(command_bytes[LEFT_WHEEL+ INDEX_OFFSET])
    right_wheel.set_wheel_speed(command_bytes[RIGHT_WHEEL + INDEX_OFFSET])
    #set gripper
    #left_gripper.move_gripper(command_bytes[LEFT_GRIPPER_SERVO + INDEX_OFFSET])
	
    servokit.setServos()


#BEGIN EYES
#eye direction constants
LOOK_CENTER = 0
LOOK_UP = 1
LOOK_DOWN = -1
LOOK_RIGHT = 1
LOOK_LEFT = -1

EYE_BLINK_DURATION = 0.1
EYE_BLINK_MIN_DELAY = 2
EYE_BLINK_CHANCE = 0.5
i2c = board.I2C()
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
    [1,1,1,1,1,1,1,1],
    [0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0]
]

command_bytes = [90] * 64
command_bytes[LEFT_GRIPPER_SERVO] = GRIPPER_OPEN_ANGLE
command_bytes[RIGHT_GRIPPER_SERVO] = 180-GRIPPER_OPEN_ANGLE # right gripper is inversed
command_bytes[LEFT_ELBOW_PITCH] = 135
command_bytes[LEFT_ELBOW_YAW] = 90
command_bytes[LEFT_SHOULDER_PITCH] = 180
command_bytes[LEFT_SHOULDER_YAW] = 135
command_bytes[HEAD_PITCH] = 90
command_bytes[HEAD_YAW] = 90
command_bytes[RIGHT_ELBOW_PITCH] = 55
command_bytes[RIGHT_ELBOW_YAW] = 90
command_bytes[RIGHT_SHOULDER_PITCH] = 0
command_bytes[RIGHT_SHOULDER_YAW] = 45
command_bytes[LEFT_WHEEL] = 0 + 127
command_bytes[RIGHT_WHEEL] = 0 + 127
command_bytes[LEFT_WRIST] = 90
command_bytes[RIGHT_WRIST] = 90

INDEX_OFFSET = 0

update_eyes = False
current_right_frame = None
current_left_frame = None
eye_direction=[LOOK_CENTER, LOOK_CENTER]
setEyeFrameByDirection(eye_direction)

#main loop
PACKET_SIGNATURE = "ALICE"
INDEX_OFFSET = len(PACKET_SIGNATURE)
command_bytes = bytearray(PACKET_SIGNATURE) + bytearray(command_bytes)
set_orientation(command_bytes)
setMatrixframe(right_eye, left_eye, current_right_frame, current_left_frame)
right_eye.show()
left_eye.show()
sleep(1)
eye_blink_timer = monotonic()
eyes_blinking = False
lastHeadMovement = None
demo=True
while True:
    loopStart = monotonic()
    if demo:
        if (lastHeadMovement == None or loopStart-lastHeadMovement > 1):
            smoothing_fraction = DEMO_SMOOTHING_FRACTION
            #this functions basically as a janky timeout, so lets stop those wheels
            #TODO: refactor to correct timeout
            left_wheel.set_wheel_speed(127) #takes signed control byte where 127 = 0 speed
            right_wheel.set_wheel_speed(127) #takes signed control byte where 127 = 0 speed
            if random() > 0.5:
                if random() > 0.3:
                    command_bytes[HEAD_PITCH + INDEX_OFFSET] = int(random()*90) + 45
                    command_bytes[HEAD_YAW + INDEX_OFFSET] = int(random()*90) + 45
                if random() > 0.67:
                    command_bytes[RIGHT_ELBOW_PITCH + INDEX_OFFSET] = 135-int(random()*90)
                    command_bytes[RIGHT_SHOULDER_PITCH + INDEX_OFFSET] = 55 + int(random()*91)
                    command_bytes[RIGHT_SHOULDER_YAW + INDEX_OFFSET] = 45 + int(random()*45)
                    command_bytes[RIGHT_WRIST + INDEX_OFFSET] = int(random()*181)
                    command_bytes[RIGHT_GRIPPER_SERVO + INDEX_OFFSET] = int(random()*90)
                if random() > 0.67:
                    command_bytes[LEFT_SHOULDER_YAW + INDEX_OFFSET] = 135 - int(random()*45)
                    command_bytes[LEFT_SHOULDER_PITCH + INDEX_OFFSET] = int(random()*91)+90
                    command_bytes[LEFT_ELBOW_PITCH + INDEX_OFFSET] = 135 - int(random()*90)
                    command_bytes[LEFT_WRIST + INDEX_OFFSET] = int(random()*181)
                    command_bytes[LEFT_GRIPPER_SERVO + INDEX_OFFSET] = 180 - int(random()*90)
            lastHeadMovement = loopStart
    packet = None
    packet = radio.receive(timeout=0)
    try:
        if packet !=None and len(packet) > 0:
            signature_check = ''
            for i in range(INDEX_OFFSET):
                signature_check+=chr(packet[i])
            if signature_check == PACKET_SIGNATURE:
                smoothing_fraction = MOCAP_SMOOTHING_FRACTION
                command_bytes = packet
    except UnicodeError as e:
        print("UnicodeError")

    set_orientation(command_bytes=command_bytes,smoothing=True)
    setEyeFrameByDirection(getEyeDirectionByOrientation())
    #blinking
    if not eyes_blinking:
        if loopStart - eye_blink_timer > EYE_BLINK_MIN_DELAY:
            if random() < EYE_BLINK_CHANCE:
                previous_left_frame = current_left_frame
                previous_right_frame = current_right_frame
                current_right_frame = frame_blink
                current_left_frame = frame_blink
                eye_blink_timer = loopStart
                eyes_blinking = True
                update_eyes = True
            else:
                #restart blink timer
                eye_blink_timer = loopStart
    elif eyes_blinking and loopStart - eye_blink_timer > EYE_BLINK_DURATION:
        current_right_frame = previous_right_frame
        current_left_frame = previous_left_frame
        eye_blink_timer = loopStart
        eyes_blinking = False
        update_eyes = True
    if update_eyes:
        setMatrixframe(right_eye, left_eye, current_right_frame, current_left_frame)
        right_eye.show()
        left_eye.show()
        update_eyes = False
    loopLength = monotonic() - loopStart
    fixedLoop = 0.03
    if loopLength < fixedLoop:
        sleep(fixedLoop - loopLength)
    #sleep(0.025 - monotonic() - loopStart)
    #print("RW:" + str(int(servokit.servo[RIGHT_WRIST].angle or -1))  + "\t\t\tREY:" + str(int(servokit.servo[RIGHT_ELBOW_YAW].angle or -1))) 