import board
import busio
import adafruit_pca9685
from time import sleep
import board
import pulseio
import digitalio
from adafruit_motor import motor
from adafruit_servokit_alice import ServoKit
import adafruit_rfm9x
from analogio import AnalogIn
 

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
bin2 = pulseio.PWMOut(board.D5)
left_wheel = motor.DCMotor(ain1,ain2)
right_wheel = motor.DCMotor(bin1,bin2)

#initialize LoRa radio
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
radio_nss = digitalio.DigitalInOut(board.A4)
radio_reset = digitalio.DigitalInOut(board.A2)
radio = adafruit_rfm9x.RFM9x(spi, radio_nss, radio_reset, 915.0)

#battery management
vbat_voltage = AnalogIn(board.VOLTAGE_MONITOR)
 
 
def get_voltage(pin):
    return (pin.value * 3.3) / 65536 * 2

#functions

def move_servo(index,command_bytes,index_offset = 0):
   servo = servokit.servo[index]
   new_angle = command_bytes[index + index_offset]
   if abs(servo.angle - new_angle) > 5:
        print('setting servo ' + str(index) + ' from ' + str(servo.angle) + ' to ' + str(new_angle))
        try:servo.angle = new_angle
        except ValueError as e: print(e)

def set_orientation(command_bytes):
    move_servo(LEFT_CLAW, command_bytes, INDEX_OFFSET)
    move_servo(LEFT_ELBOW_PITCH, command_bytes, INDEX_OFFSET)
    move_servo(LEFT_ELBOW_YAW, command_bytes, INDEX_OFFSET)
    move_servo(LEFT_SHOULDER_PITCH, command_bytes, INDEX_OFFSET)
    move_servo(LEFT_SHOULDER_YAW, command_bytes, INDEX_OFFSET)
    move_servo(HEAD_PITCH, command_bytes, INDEX_OFFSET)
    move_servo(HEAD_YAW, command_bytes, INDEX_OFFSET)
    move_servo(RIGHT_CLAW, command_bytes, INDEX_OFFSET)
    move_servo(RIGHT_ELBOW_PITCH, command_bytes, INDEX_OFFSET)
    move_servo(RIGHT_ELBOW_YAW, command_bytes, INDEX_OFFSET)
    move_servo(RIGHT_SHOULDER_PITCH, command_bytes, INDEX_OFFSET)
    move_servo(RIGHT_SHOULDER_YAW, command_bytes, INDEX_OFFSET)

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
command_bytes[RIGHT_SHOULDER_YAW] = 90

INDEX_OFFSET = 0

set_orientation(command_bytes)

#main loop
PACKET_SIGNATURE = "ALICE"
INDEX_OFFSET = len(PACKET_SIGNATURE)

while True:
    packet = radio.receive()  # Wait for a packet to be received (up to 0.5 seconds)
    battery_voltage = get_voltage(vbat_voltage)
    print("VBat voltage: {:.2f}".format(battery_voltage))
    if packet is not None:
         try:
            #packet_text = str(packet, 'ascii')
            if len(packet) > 0: 
                print('Received: {0}'.format(str(packet)))
                command_bytes = packet
                signature_check = ''
                for i in range(INDEX_OFFSET):
                    signature_check+=chr(packet[i])
                print('signature:' + signature_check)
                if signature_check == PACKET_SIGNATURE:
                    set_orientation(command_bytes)
         except UnicodeError as e:
             print("UnicodeError")
    sleep(0.05)