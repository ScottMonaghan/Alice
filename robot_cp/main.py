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
 
#constants
MIN_ANGLE_CHANGE = 3
SMOOTHING_ANGLE = 45
MAX_SPEED = 0.5
DELAY_THROTTLE = 0.4
MIN_SPEED = 0.35
MAX_WHEEL_DELAY = 10

#gripper_constants
GRIPPER_TIGHTNESS = 30
GRIPPER_TOLERANCE = 5
GRIPPER_MOTION = 10
GRIPPER_SERVO = 10
GRIPPER_OPEN_ANGLE = 0
GRIPPER_CLOSE_ANGLE = 90
GRIPPER_CLOSE_EASE = 0.40

#initialize servos for arms and head
LEFT_CLAW = 0
LEFT_ELBOW_PITCH = 1
LEFT_ELBOW_YAW = 2
LEFT_SHOULDER_PITCH = 3
LEFT_SHOULDER_YAW = 4
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
        #self.delayStartTime = monotonic()
        self.delayLoops = 0
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
            delay = int(MAX_WHEEL_DELAY * ((MIN_SPEED - math.fabs(speed))/float(MIN_SPEED)))
            if self.delayLoops >= delay:
               #we're past the delay, send a pulse at MIN_SPEED and start the delay clock again
                self.motor.throttle = DELAY_THROTTLE
                self.delayLoops = 0
            else:
                self.motor.throttle = 0
                self.delayLoops+=1
        else:
            self.motor.throttle = speed

servokit = ServoKit(channels=16)

##initialize gripper
right_gripper_sensor = DigitalInOut(board.D5)
right_gripper_sensor.direction = Direction.INPUT
right_gripper_sensor.pull = Pull.DOWN
right_gripper = Gripper(servokit.servo[GRIPPER_SERVO],right_gripper_sensor)

##initialize wheel motors
wheels_sleep = DigitalInOut(board.D12)
wheels_sleep.direction = Direction.OUTPUT
wheels_sleep.value = True
ain1 = pulseio.PWMOut(board.D11)
ain2 = pulseio.PWMOut(board.D10)
bin1 = pulseio.PWMOut(board.D6)
bin2 = pulseio.PWMOut(board.D9)
left_wheel_motor = motor.DCMotor(ain1,ain2)
right_wheel_motor = motor.DCMotor(bin1,bin2)
left_wheel_motor.throttle = 0
right_wheel_motor.throttle = 0
left_wheel = Wheel(left_wheel_motor)
right_wheel = Wheel(right_wheel_motor)

#initialize LoRa radio
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
radio_nss = DigitalInOut(board.A4)
radio_reset = DigitalInOut(board.A2)
radio = adafruit_rfm9x.RFM9x(spi, radio_nss, radio_reset, 915.0)

#battery management
vbat_voltage = AnalogIn(board.VOLTAGE_MONITOR)
 

		
def get_voltage(pin):
    return (pin.value * 3.3) / 65536 * 2

#functions

def move_servo(index,command_bytes,index_offset = 0, smoothing=False):
   servo = servokit.servo[index]
   new_angle = command_bytes[index + index_offset]
   if abs(servo.angle - new_angle) > MIN_ANGLE_CHANGE:
        try:
            if (smoothing):
                smooth_angle = (new_angle - servo.angle) * 0.5
                if (smooth_angle > SMOOTHING_ANGLE):
                    smooth_angle = SMOOTHING_ANGLE
                if (smooth_angle < -1*SMOOTHING_ANGLE):
                    smooth_angle = -1*SMOOTHING_ANGLE
                servo.angle += smooth_angle
            else:
                servo.angle = new_angle
            
        except ValueError as e: print(e)

def set_orientation(command_bytes, smoothing = False):
    #move arms and head
    move_servo(LEFT_CLAW, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(LEFT_ELBOW_PITCH, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(LEFT_ELBOW_YAW, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(LEFT_SHOULDER_PITCH, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(LEFT_SHOULDER_YAW, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(HEAD_PITCH, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(HEAD_YAW, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(RIGHT_WRIST, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(RIGHT_ELBOW_PITCH, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(RIGHT_ELBOW_YAW, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(RIGHT_SHOULDER_PITCH, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(RIGHT_SHOULDER_YAW, command_bytes, INDEX_OFFSET,smoothing)

    #set wheel speed
    left_wheel.set_wheel_speed(command_bytes[LEFT_WHEEL+ INDEX_OFFSET])
    right_wheel.set_wheel_speed(command_bytes[RIGHT_WHEEL + INDEX_OFFSET])
    #set gripper
    right_gripper.move_gripper(command_bytes[GRIPPER_SERVO + INDEX_OFFSET]) 
	
    servokit.setServos()
command_bytes = [90] * 64

command_bytes[LEFT_CLAW] = 90
command_bytes[LEFT_ELBOW_PITCH] = 90
command_bytes[LEFT_ELBOW_YAW] = 90
command_bytes[LEFT_SHOULDER_PITCH] = 90
command_bytes[LEFT_SHOULDER_YAW] = 90
command_bytes[HEAD_PITCH] = 45
command_bytes[HEAD_YAW] = 90
command_bytes[RIGHT_WRIST] = 45
command_bytes[RIGHT_ELBOW_PITCH] = 90
command_bytes[RIGHT_ELBOW_YAW] = 90
command_bytes[RIGHT_SHOULDER_PITCH] = 0
command_bytes[RIGHT_SHOULDER_YAW] = 45
command_bytes[LEFT_WHEEL] = 0 + 127
command_bytes[RIGHT_WHEEL] = 0 + 127
command_bytes[GRIPPER_SERVO] = GRIPPER_OPEN_ANGLE


INDEX_OFFSET = 0

#main loop
PACKET_SIGNATURE = "ALICE"
INDEX_OFFSET = len(PACKET_SIGNATURE)
command_bytes = bytearray(PACKET_SIGNATURE) + bytearray(command_bytes)
set_orientation(command_bytes)
while True:
    loopStart = monotonic()
    packet = None
    packet = radio.receive(timeout=0)  # Wait for a packet to be received (up to 0.5 seconds)
    if packet is not None:
         try:
            if len(packet) > 0: 
                signature_check = ''
                for i in range(INDEX_OFFSET):
                    signature_check+=chr(packet[i])
                if signature_check == PACKET_SIGNATURE:
                    command_bytes = packet
         except UnicodeError as e:
             print("UnicodeError")
    set_orientation(command_bytes=command_bytes,smoothing=True)
    print (str(monotonic() - loopStart))