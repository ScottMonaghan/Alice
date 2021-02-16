import sys
import time
import board
import busio
import math
import digitalio
import struct
from scottsrobots_cp_quaternion import Quaternion
from scottsrobots_body_tracker import Joint, TrackedBody

def log_error(exc):
    sys.print_exception(exc)

#initialize command for arms and head
COMMAND_LENGTH = 20
#gripper_constants


#initialize servos for arms and head
LEFT_GRIPPER_SERVO = 0
LEFT_ELBOW_PITCH = 1
LEFT_ELBOW_YAW = 2
LEFT_SHOULDER_PITCH = 3
LEFT_SHOULDER_YAW = 4
LEFT_WRIST = 5
HEAD_YAW = 7
HEAD_PITCH =8
RIGHT_WRIST = 10
RIGHT_GRIPPER_SERVO = 11
RIGHT_ELBOW_PITCH = 12
RIGHT_ELBOW_YAW = 13
RIGHT_SHOULDER_PITCH = 14
RIGHT_SHOULDER_YAW = 15


LEFT_WHEEL = 16
RIGHT_WHEEL = 17

HEADING_START = 18
HEADING_STOP = 19
HEADING_MULTIPLIER = 100

CONFIG_FLAGS = 20
PACKET_SIGNATURE = "ALICE"
INDEX_OFFSET = len(PACKET_SIGNATURE)

PITCH_FORWARD_MIN = -10
PITCH_FORWARD_MAX = -15

PITCH_REVERSE_MIN = 10
PITCH_REVERSE_MAX = 15

#initialize bno085
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
i2c = busio.I2C(board.SCL, board.SDA, frequency=10000)
bno_chest = BNO08X_I2C(i2c)
bno_chest.enable_feature(BNO_REPORT_ROTATION_VECTOR)

#initialize rfm95
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.A5)
reset = digitalio.DigitalInOut(board.A4)
import adafruit_rfm9x
rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, 915.0)

command_bytes = [90] * COMMAND_LENGTH
command_bytes[LEFT_GRIPPER_SERVO] = 90
command_bytes[RIGHT_GRIPPER_SERVO] = 90
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

signed_command = bytearray(PACKET_SIGNATURE) + bytearray(command_bytes)

#initialize body
tracked_body = TrackedBody()
#assign bno trackers to body joints
tracked_body.joint_chest.bno085 = bno_chest
#wait five seconds to initialize
print("initializing body tracker in 5 seconds...")
time.sleep(5)
#initilize body joint orientations
tracked_body.initialize_orientations()
print("Entering main loop..")
time.sleep(1)
while True:
    try:
        tracked_body.update_orientations()
        #set speed and heading based on chest yaw and pitch
        chest_euler = Quaternion.to_euler(tracked_body.joint_chest.offset_orientation)
        command_bytes = bytearray(command_bytes)
        struct.pack_into('>H',command_bytes, HEADING_START, int(chest_euler.yaw * HEADING_MULTIPLIER))
        pitch = chest_euler.pitch
        speed_command_byte = 127
        if pitch < PITCH_FORWARD_MIN:
            if pitch < PITCH_FORWARD_MAX: pitch = PITCH_FORWARD_MAX
            speed_command_byte = 127 + int(127 * ((math.fabs(pitch)-math.fabs(PITCH_FORWARD_MIN))/(math.fabs(PITCH_FORWARD_MAX) - math.fabs(PITCH_FORWARD_MIN))))
        if pitch > PITCH_REVERSE_MIN:
            if pitch > PITCH_REVERSE_MAX: pitch = PITCH_REVERSE_MAX
            speed_command_byte = 127 - int(127 * ((pitch-PITCH_REVERSE_MIN)/(PITCH_REVERSE_MAX-PITCH_REVERSE_MIN)))
        command_bytes[LEFT_WHEEL]=speed_command_byte
        command_bytes[RIGHT_WHEEL]=speed_command_byte
        signed_command = bytearray(PACKET_SIGNATURE) + bytearray(command_bytes)
        packet = rfm9x.send(signed_command)
        print("chest_euler.pitch: " + str(chest_euler.pitch) + "\t\tchest_euler.yaw: " + str(chest_euler.yaw))
        #time.sleep(0.1)
    except Exception as e:
        log_error(e)
        time.sleep(0.1)