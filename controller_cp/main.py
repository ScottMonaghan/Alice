import board
import busio
import digitalio
from supervisor import runtime
from time import sleep
import adafruit_bno055
from digitalio import DigitalInOut, Direction, Pull
from analogio import AnalogIn
from adafruit_motor import motor
from pulseio import PWMOut

#constants
COMMAND_LENGTH = 20
HEAD_YAW_START = 90
HEAD_YAW_MAX = 180
HEAD_YAW_MIN = 0
HEAD_PITCH_START = 90
HEAD_PITCH_MAX = 180
HEAD_PITCH_MIN = 0
LEFT_CLAW_OPEN = 180
LEFT_CLAW_CLOSED = 0
RIGHT_CLAW_OPEN = 135
RIGHT_CLAW_CLOSED = 0

#initialize servos for arms and head
LEFT_CLAW = 0
LEFT_ELBOW_PITCH = 1
LEFT_ELBOW_YAW = 2
LEFT_SHOULDER_PITCH = 3
LEFT_SHOULDER_YAW = 4
RIGHT_WRIST = 11
RIGHT_CLAW = 10
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
cs = digitalio.DigitalInOut(board.D6) #DigitalInOut(board.A0)
reset = digitalio.DigitalInOut(board.D9) #.DigitalInOut(board.A1)
import adafruit_rfm9x
rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, 915.0)

#init BNO055 orientation tracking
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)
yawOffsetResetButton = DigitalInOut(board.A2)
yawOffsetResetButton.direction = Direction.INPUT
yawOffsetResetButton.pull = Pull.UP
bno055yawStartingPosition = -1 #initialize variable

leftGripperButton = DigitalInOut(board.A3)
leftGripperButton.direction = Direction.INPUT
leftGripperButton.pull = Pull.UP
leftGripperButtonDebounce = False
leftGripperClosed = False

rightGripperButton = DigitalInOut(board.D10)
rightGripperButton.direction = Direction.INPUT
rightGripperButton.pull = Pull.UP
rightGripperButtonDebounce = False
rightGripperClosed = False

leftRightStick_left = AnalogIn(board.A4)
forwardBackStick_left = AnalogIn(board.A5)

forwardBackStick_right = AnalogIn(board.A0)
leftRightStick_right = AnalogIn(board.A1)

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
        distance = (360 - startingPosition) + position
    
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

def processForwardBackStick(stick):
    forwardBackStickMin = 256
    forwardBackStickMax = 65520
    forwardBackStickNeutralMax = 35000
    forwardBackStickNeutralMin = 31000
    forwardBackStickValue = stick.value
    maxSpeed = 0.5

    #figure out forward speed
    if forwardBackStickValue > forwardBackStickMax: forwardBackStickValue = forwardBackStickMax
    if forwardBackStickValue < forwardBackStickMin: forwardBackStickValue = forwardBackStickMin

    forwardBackPct = 0.5
    if forwardBackStickValue > forwardBackStickNeutralMax or forwardBackStickValue < forwardBackStickNeutralMin:
        #inverted because forward is min and back is max due to physical orientation of stick
        forwardBackPct = 1.0 - (float((forwardBackStickValue - forwardBackStickMin))/(float(forwardBackStickMax-forwardBackStickMin)))
    return (int(forwardBackPct * 254 * maxSpeed) - int
    
    (127 * maxSpeed))

def processLeftRightStick(stick):
    min = 300
    max = 65520
    rightStart = 35000
    leftStart = 31000
    stickValue = stick.value
    maxTurnSpeed = 0.3

    if stickValue > max: stickValue = max
    if stickValue < min: stickValue = min

    #pct neutral = 0
    #pct > 0 = right
    #pct < 0 = left
    pct = 0

    #right 
    if stickValue > rightStart:
        pct = float(stickValue - rightStart) / float(max - rightStart)
    if stickValue < leftStart:
        pct = float(leftStart - stickValue) / float(leftStart-min) * -1
    return round(pct * 127 * maxTurnSpeed)
    
def processGripperButtons():
    global leftGripperButtonDebounce
    global leftGripperClosed
    global rightGripperButtonDebounce
    global rightGripperClosed
    leftButtonPressed = not leftGripperButton.value #False when button pressed
    rightButtonPressed = not rightGripperButton.value

    if leftButtonPressed and not leftGripperButtonDebounce:
        leftGripperButtonDebounce = True
        #toggle gripper
        leftGripperClosed = not leftGripperClosed
    elif not leftButtonPressed and leftGripperButtonDebounce:
        leftGripperButtonDebounce = False

    if rightButtonPressed and not rightGripperButtonDebounce:
        rightGripperButtonDebounce = True
        #toggle gripper
        rightGripperClosed = not rightGripperClosed
    elif not rightButtonPressed and rightGripperButtonDebounce:
        rightGripperButtonDebounce = False   
    
command_bytes = [90] * COMMAND_LENGTH
command_bytes[LEFT_CLAW] = 180
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
        #now process data from joystick
        command_bytes[RIGHT_WHEEL] = 0 + 127
        command_bytes[LEFT_WHEEL] = 0 + 127
        forwardBackSpeed = processForwardBackStick(forwardBackStick_left)
        if forwardBackSpeed == 0:
            forwardBackSpeed = processForwardBackStick(forwardBackStick_right) 

        if forwardBackSpeed != 0:
            command_bytes[RIGHT_WHEEL] = forwardBackSpeed + 127
            command_bytes[LEFT_WHEEL] = forwardBackSpeed + 127
        
        leftRightSpeed = processLeftRightStick(leftRightStick_left)
        if leftRightSpeed == 0:
            leftRightSpeed = processLeftRightStick(leftRightStick_right)
        if leftRightSpeed != 0:
            command_bytes[RIGHT_WHEEL] = (-1 * leftRightSpeed) + 127
            command_bytes[LEFT_WHEEL] = leftRightSpeed + 127

        processGripperButtons()
        if leftGripperClosed:
            command_bytes[LEFT_CLAW] = LEFT_CLAW_CLOSED
        else:
            command_bytes[LEFT_CLAW] = LEFT_CLAW_OPEN

        if rightGripperClosed:
            command_bytes[RIGHT_CLAW] = RIGHT_CLAW_CLOSED
        else:
            command_bytes[RIGHT_CLAW] = RIGHT_CLAW_OPEN

        convertedHeadAngles = convertBNO055EulerToHeadAngles(sensor.euler,bno055yawStartingPosition)
        command_bytes[HEAD_YAW] = convertedHeadAngles[0]
        command_bytes[HEAD_PITCH] = convertedHeadAngles[1]
        signed_command = bytearray(PACKET_SIGNATURE) + bytearray(command_bytes)
        packet = rfm9x.send(signed_command)
    else:
        sleep(0.1)
    
    