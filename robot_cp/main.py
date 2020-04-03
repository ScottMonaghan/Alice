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
MIN_ANGLE_CHANGE = 3
SMOOTHING_ANGLE = 45
MAX_SPEED = 0.5
MIN_SPEED = 0.35



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

LEFT_WHEEL = 16
RIGHT_WHEEL = 17

servokit = ServoKit(channels=16)

##initialize wheel motors
wheels_sleep = DigitalInOut(board.D12)
wheels_sleep.direction = Direction.OUTPUT
wheels_sleep.value = True
ain1 = pulseio.PWMOut(board.D11)
ain2 = pulseio.PWMOut(board.D10)
bin1 = pulseio.PWMOut(board.D6)
bin2 = pulseio.PWMOut(board.D9)
left_wheel = motor.DCMotor(ain1,ain2)
right_wheel = motor.DCMotor(bin1,bin2)
left_wheel.throttle = 0
right_wheel.throttle = 0

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
        #print('setting servo ' + str(index) + ' from ' + str(servo.angle) + ' to ' + str(new_angle))
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
    move_servo(LEFT_CLAW, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(LEFT_ELBOW_PITCH, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(LEFT_ELBOW_YAW, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(LEFT_SHOULDER_PITCH, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(LEFT_SHOULDER_YAW, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(HEAD_PITCH, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(HEAD_YAW, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(RIGHT_CLAW, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(RIGHT_ELBOW_PITCH, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(RIGHT_ELBOW_YAW, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(RIGHT_SHOULDER_PITCH, command_bytes, INDEX_OFFSET, smoothing)
    move_servo(RIGHT_SHOULDER_YAW, command_bytes, INDEX_OFFSET,smoothing)
    # set wheel speed
    speed_range = MAX_SPEED - MIN_SPEED
    left_speed = ((command_bytes[LEFT_WHEEL+ INDEX_OFFSET] - 127)/127.0) * speed_range
    right_speed = ((command_bytes[RIGHT_WHEEL+ INDEX_OFFSET] - 127)/127.0) * speed_range
    if left_speed < 0: left_speed-=MIN_SPEED
    elif left_speed > 0: left_speed+=MIN_SPEED
    if right_speed < 0: right_speed-=MIN_SPEED
    elif right_speed > 0: right_speed+=MIN_SPEED
    
    print(left_speed)
    
    if math.fabs(left_speed) < MIN_SPEED:
        left_speed = 0
    if math.fabs(right_speed) < MIN_SPEED:
        right_speed = 0
    left_wheel.throttle = left_speed
    right_wheel.throttle = right_speed
    #print(left_wheel.throttle)
command_bytes = [90] * 64

command_bytes[LEFT_CLAW] = 90
command_bytes[LEFT_ELBOW_PITCH] = 90
command_bytes[LEFT_ELBOW_YAW] = 90
command_bytes[LEFT_SHOULDER_PITCH] = 90
command_bytes[LEFT_SHOULDER_YAW] = 90
command_bytes[HEAD_PITCH] = 90
command_bytes[HEAD_YAW] = 90
command_bytes[RIGHT_CLAW] = 90
command_bytes[RIGHT_ELBOW_PITCH] = 90
command_bytes[RIGHT_ELBOW_YAW] = 90
command_bytes[RIGHT_SHOULDER_PITCH] = 90
command_bytes[RIGHT_SHOULDER_YAW] = 45
command_bytes[LEFT_WHEEL] = 0 + 127
command_bytes[RIGHT_WHEEL] = 0 + 127
INDEX_OFFSET = 0

#set_orientation(command_bytes)

#main loop
PACKET_SIGNATURE = "ALICE"
INDEX_OFFSET = len(PACKET_SIGNATURE)
command_bytes = bytearray(PACKET_SIGNATURE) + bytearray(command_bytes)
set_orientation(command_bytes)
while True:
    packet = radio.receive(timeout=0.01)  # Wait for a packet to be received (up to 0.5 seconds)
    battery_voltage = get_voltage(vbat_voltage)
    #print("VBat voltage: {:.2f}".format(battery_voltage))
    if packet is not None:
         try:
            #packet_text = str(packet, 'ascii')
            if len(packet) > 0: 
                #print('Received: {0}'.format(str(packet)))
                signature_check = ''
                for i in range(INDEX_OFFSET):
                    signature_check+=chr(packet[i])
                #print('signature:' + signature_check)
                if signature_check == PACKET_SIGNATURE:
                    command_bytes = packet
         except UnicodeError as e:
             print("UnicodeError")
    #print(int(command_bytes[LEFT_WHEEL+INDEX_OFFSET]))
    set_orientation(command_bytes=command_bytes,smoothing=True)
    #sleep(0.001)