import board
import busio
import digitalio
from supervisor import runtime
from time import sleep
import adafruit_bno055
from digitalio import DigitalInOut, Direction, Pull
from adafruit_motor import motor
from pulseio import PWMOut

#constants
COMMAND_LENGTH = 20
HEAD_YAW_START = 90
HEAD_YAW_MAX = 165
HEAD_YAW_MIN = 15
HEAD_PITCH_START = 90
HEAD_PITCH_MAX = 180
HEAD_PITCH_MIN = 0

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
PACKET_SIGNATURE = "ALICE"
INDEX_OFFSET = len(PACKET_SIGNATURE)

#init RF95
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.A0)
reset = digitalio.DigitalInOut(board.A1)
import adafruit_rfm9x
rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, 915.0)

#init BNO055 orientation tracking
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)
yawOffsetResetButton = DigitalInOut(board.A2)
yawOffsetResetButton.direction = Direction.INPUT
yawOffsetResetButton.pull = Pull.UP
bno055yawStartingPosition = -1 #initialize variable


def getDistanceFromStartingPostion(position, startingPosition):
    #need to figure out how to return the positive or negative angle differece given that 0-1 = 359.
    #so what do I know?
    #all angles are +/- 180 from the startingPostion. So if any angle is greater than -180 or 180 away, then we need to convert
    
    #start with the basics
    distance = position - startingPosition
    
    #e.g. startingPosition = 15, position = 340, distance transforms from 325 to -35
    if (distance > 180):
        distance = position - 360 - startingPosition
    
    #e.g. startingPosition = 325, position = 5, distance transforms from -320 to 40
    if (distance < -180):
        distance = (360 - startingPostion) + position
    
    return distance
    
def convertBNO055EulerToHeadAngles(sensorAngles,sensorYawZeroPosition):
    sensorYaw = int(sensorAngles[0])
    sensorPitch = int(sensorAngles[1])
    
    #note the servos are inversed
    #now get the head yaw
    headYaw = 180 - (HEAD_YAW_START + getDistanceFromStartingPostion(sensorYaw, sensorYawZeroPosition));
    if (headYaw > HEAD_YAW_MAX):
        headYaw = HEAD_YAW_MAX
    
    if (headYaw < HEAD_YAW_MIN):
        headYaw = HEAD_YAW_MIN
    
    
    #now get head pitch
    headPitch = 180 - (HEAD_PITCH_START -  sensorPitch)
    if (headPitch > HEAD_PITCH_MAX):
        headPitch = HEAD_PITCH_MAX
        
    if (headPitch < HEAD_PITCH_MIN):
        headPitch = HEAD_PITCH_MIN
    
    return (headYaw,headPitch)

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
command_bytes[LEFT_WHEEL] = 0 + 127
command_bytes[RIGHT_WHEEL] = 0 + 127

signed_command = bytearray(PACKET_SIGNATURE) + bytearray(command_bytes)

while True:
    #first check to see if we received new data from the kinect
    if runtime.serial_bytes_available:
        new_command = bytearray(eval("b'" + input() + "'"))
        if len(new_command) == COMMAND_LENGTH:
            #signed_command = bytearray(PACKET_SIGNATURE) + new_command
            command_bytes = new_command
    #now process data from the BNO055
    if (not yawOffsetResetButton.value):
        bno055yawStartingPosition = int(sensor.euler[0])
    elif (bno055yawStartingPosition >=0):
        convertedHeadAngles = convertBNO055EulerToHeadAngles(sensor.euler,bno055yawStartingPosition)
        command_bytes[HEAD_YAW] = convertedHeadAngles[0]
        command_bytes[HEAD_PITCH] = convertedHeadAngles[1]
        signed_command = bytearray(PACKET_SIGNATURE) + bytearray(command_bytes)
        packet = rfm9x.send(signed_command)
   #  print(
#         int(command_bytes[LEFT_WHEEL])
#         )