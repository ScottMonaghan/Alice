import board
import busio
import digitalio
from supervisor import runtime
from time import sleep

#constants
COMMAND_LENGTH = 20

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
PACKET_SIGNATURE = "ALICE"
INDEX_OFFSET = len(PACKET_SIGNATURE)


spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.D1)
reset = digitalio.DigitalInOut(board.D0)
import adafruit_rfm9x
rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, 915.0)

command_bytes = [90] * COMMAND_LENGTH
command_bytes[LEFT_CLAW] = 90
command_bytes[LEFT_ELBOW_PITCH] = 45
command_bytes[LEFT_ELBOW_YAW] = 135
command_bytes[LEFT_SHOULDER_PITCH] = 180
command_bytes[LEFT_SHOULDER_YAW] = 90
command_bytes[HEAD_PITCH] = 90
command_bytes[HEAD_YAW] = 90
command_bytes[RIGHT_CLAW] = 90
command_bytes[RIGHT_ELBOW_PITCH] = 90
command_bytes[RIGHT_ELBOW_YAW] = 45
command_bytes[RIGHT_SHOULDER_PITCH] = 0
command_bytes[RIGHT_SHOULDER_YAW] = 90

signed_command = bytearray(PACKET_SIGNATURE) + bytearray(command_bytes)

while True:
    if runtime.serial_bytes_available:
        #print("data received")
        new_command = bytearray(eval("b'" + input() + "'"))
        if len(new_command) == COMMAND_LENGTH:
            signed_command = bytearray(PACKET_SIGNATURE) + new_command
        #else:
            #print(len(new_command))
    packet = rfm9x.send(signed_command)
    #print(str(signed_command))
    #sleep(0.01)