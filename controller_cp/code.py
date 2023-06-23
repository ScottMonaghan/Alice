import sys
import time
import board
import busio
import math
import digitalio
import struct
from adafruit_tca9548a import TCA9548A, TCA9548A_Channel
from scottsrobots_cp_quaternion import Quaternion, EulerOrientation
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
HEAD_YAW = 8
HEAD_PITCH =7
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

#initialize rfm95
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.D6)
reset = digitalio.DigitalInOut(board.D9)
import adafruit_rfm9x
rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, 915.0)


#initialize bno085s
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GAME_ROTATION_VECTOR
)

from adafruit_bno08x.i2c import BNO08X_I2C
i2c = busio.I2C(board.SCL, board.SDA)#, frequency=10000)
tca = TCA9548A(i2c)
#left arm i2c bus plugged into tca channel 0
bno_chest = None
try:
    bno_chest = BNO08X_I2C(tca[0])
    bno_chest.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
except Exception as e:
    try:raise ValueError("Chest bno085 not connected")from e
    except ValueError as custom_exc: log_error(custom_exc)

bno_head = None
try:
    bno_head = BNO08X_I2C(tca[7], address=0x4B)
    bno_head.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
except Exception as e:
    try:raise ValueError("Head bno085 not connected")from e
    except ValueError as custom_exc: log_error(custom_exc)

bno_left_shoulder = None
try:
    bno_left_shoulder = BNO08X_I2C(tca[2],address=0x4B)
    bno_left_shoulder.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
except Exception as e:
    try:raise ValueError("Left Shoulder bno085 not connected")from e
    except ValueError as custom_exc: log_error(custom_exc)

bno_left_forearm = None
# try:
#     bno_left_forearm = BNO08X_I2C(tca[1], address=0x4B)
#     bno_left_forearm.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
# except Exception as e:
#     try:raise ValueError("Left Forearm bno085 not connected")from e
#     except ValueError as custom_exc: log_error(custom_exc)

bno_left_wrist = None
try:
    bno_left_wrist = BNO08X_I2C(tca[2])
    bno_left_wrist.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
except Exception as e:
    try:raise ValueError("Left Wrist bno085 not connected")from e
    except ValueError as custom_exc: log_error(custom_exc)


command_bytes = [90] * COMMAND_LENGTH
command_bytes[LEFT_GRIPPER_SERVO] = 90
command_bytes[RIGHT_GRIPPER_SERVO] = 90
command_bytes[LEFT_ELBOW_PITCH] = 180
command_bytes[LEFT_ELBOW_YAW] = 90
command_bytes[LEFT_SHOULDER_PITCH] = 60
command_bytes[LEFT_SHOULDER_YAW] = 90
command_bytes[HEAD_PITCH] = 90
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
if bno_chest is not None:
    tracked_body.joint_chest.bno085 = bno_chest
if bno_head is not None:
    tracked_body.joint_head.bno085 = bno_head
if bno_left_shoulder is not None:
    tracked_body.joint_left_shoulder.bno085 = bno_left_shoulder
if bno_left_forearm is not None:
    tracked_body.joint_left_forearm.bno085 = bno_left_forearm
if bno_left_wrist is not None:
    tracked_body.joint_left_wrist.bno085 = bno_left_wrist
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

        #set head pitch and yaw based on adjusted head yaw and pitch
        head_yaw_euler = Quaternion.to_euler(tracked_body.joint_head.servo_orientation)
        head_pitch_euler = Quaternion.to_euler(tracked_body.joint_head.raw_orientation)
        tracked_head_yaw = int(head_yaw_euler.yaw)
        tracked_head_pitch = int(head_pitch_euler.pitch)
        #figure out left arm orientations
        #for the left shoulder we need to translate the following:
        #   sensor yaw: shoulder yaw servo
        #   sensor pitch: shoulder pitch servo
        #   sensor roll: elbow yaw servo

        #first figure out left shoulder yaw servo value
        #get basic value
        joint_left_shoulder_euler = Quaternion.to_euler(tracked_body.joint_left_shoulder.servo_orientation)
        left_shoulder_yaw_servo_value = joint_left_shoulder_euler.yaw
        #neutral position starts at 0 but we want it to start at 90
        left_shoulder_yaw_servo_value = EulerOrientation.add_angles(90,left_shoulder_yaw_servo_value)
        #next we need to limit the range to a max of 180 and min of 45
        if left_shoulder_yaw_servo_value > 180:
            left_shoulder_yaw_servo_value = 180
        elif left_shoulder_yaw_servo_value < 45:
            left_shoulder_yaw_servo_value = 45
        #finally we need trim to an integer
        left_shoulder_yaw_servo_value = int(left_shoulder_yaw_servo_value)

        #now figure out the left shoulder pitch servo value
        left_shoulder_pitch_servo_value = joint_left_shoulder_euler.pitch
        #neutral position starts at 0 but we want it to start at 90
        left_shoulder_pitch_servo_value = EulerOrientation.add_angles(90,left_shoulder_pitch_servo_value)
        #in the case of pitch, the servo full-range is 0-180
        #   but the sensor doesn't record that range in practice
        #   so we'll actually want to use a proportional range to
        #   make sure we can take advantage of the full servo range
        MAX_SHOULDER_PITCH_SENSOR_VALUE = 160
        MIN_SHOULDER_PITCH_SENSOR_VALUE = 20
        left_shoulder_pitch_servo_value = (
            (left_shoulder_pitch_servo_value-MIN_SHOULDER_PITCH_SENSOR_VALUE)
            /(MAX_SHOULDER_PITCH_SENSOR_VALUE-MIN_SHOULDER_PITCH_SENSOR_VALUE)
            )*180
        #next we need to limit the range to a max of 180 and min of 0
        if left_shoulder_pitch_servo_value > 180:
            left_shoulder_pitch_servo_value = 180
        elif left_shoulder_pitch_servo_value < 0:
            left_shoulder_pitch_servo_value = 0
        #inverse from 180 for the  servo
        left_shoulder_pitch_servo_value = 180 - left_shoulder_pitch_servo_value
        #finally we need trim to an integer
        left_shoulder_pitch_servo_value = int(left_shoulder_pitch_servo_value)

        #now figure out the left elbow yaw servo value (left shoulder sensor roll)
        left_elbow_yaw_servo_value = joint_left_shoulder_euler.roll
        #for the left elbo yaw servo we want the angle to increase clockwise (opposite of sensor)
        left_elbow_yaw_servo_value = 180 - left_elbow_yaw_servo_value
        #in the case of elbow_yaw, the servo full-range is 45-180
        #   but the sensor doesn't record that range in practice
        #   so we'll actually want to use a proportional range to
        #   make sure we can take advantage of the full servo range
        MAX_SHOULDER_ROLL_SENSOR_VALUE = 180
        MIN_SHOULDER_ROLL_SENSOR_VALUE = 100
        #next we need to limit the range to a max of 180 and min of 0
        if left_elbow_yaw_servo_value > MAX_SHOULDER_ROLL_SENSOR_VALUE:
            left_elbow_yaw_servo_value = MAX_SHOULDER_ROLL_SENSOR_VALUE
        elif left_elbow_yaw_servo_value < MIN_SHOULDER_ROLL_SENSOR_VALUE:
            left_elbow_yaw_servo_value = MIN_SHOULDER_ROLL_SENSOR_VALUE

        left_elbow_yaw_servo_value = (
            (left_elbow_yaw_servo_value-MIN_SHOULDER_ROLL_SENSOR_VALUE)
            /(MAX_SHOULDER_ROLL_SENSOR_VALUE-MIN_SHOULDER_ROLL_SENSOR_VALUE)
            )*(180-45) + 45
        #finally we need trim to an integer
        left_elbow_yaw_servo_value = int(left_elbow_yaw_servo_value)

        #for the left wrist we need to translate the following:
        #   sensor yaw: elbow pitch servo
        #   sensor pitch: (alternate) elbow yaw servo
        #   sensor roll: wrist servo

        #first figure out left elbow pitch servo value

        #get basic value
        joint_left_wrist_euler = Quaternion.to_euler(tracked_body.joint_left_wrist.servo_orientation)
        left_elbow_pitch_servo_value = joint_left_wrist_euler.yaw
        #for the left elbo pitch servo we want to start at 180, not 0/360 like the sensor
        left_elbow_pitch_servo_value = EulerOrientation.add_angles(180, left_elbow_pitch_servo_value)
        #next we need to limit the range to a max of 180 and min of 0
        if left_elbow_pitch_servo_value > 180:
            left_elbow_pitch_servo_value = 180
        elif left_elbow_pitch_servo_value < 0:
            left_elbow_pitch_servo_value = 0
        #finally we need trim to an integer
        left_elbow_pitch_servo_value = int(left_elbow_pitch_servo_value)

        #first figure out left elbow pitch servo value
        left_wrist_servo_value = joint_left_wrist_euler.roll
        #add 90 degrees to match servo starting value
        left_wrist_servo_value = EulerOrientation.add_angles(90, left_wrist_servo_value)

        MAX_WRIST_ROLL_SENSOR_VALUE = 180
        MIN_WRIST_ROLL_SENSOR_VALUE = 45
        #next we need to limit the range to a max of 180 and min of 0
        if left_wrist_servo_value > MAX_WRIST_ROLL_SENSOR_VALUE:
            left_wrist_servo_value = MAX_WRIST_ROLL_SENSOR_VALUE
        elif left_wrist_servo_value < MIN_WRIST_ROLL_SENSOR_VALUE:
            left_wrist_servo_value = MIN_WRIST_ROLL_SENSOR_VALUE

        left_wrist_servo_value = (
            (left_wrist_servo_value-MIN_WRIST_ROLL_SENSOR_VALUE)
            /(MAX_WRIST_ROLL_SENSOR_VALUE-MIN_WRIST_ROLL_SENSOR_VALUE)
            )*180

        #now inverse!
        left_wrist_servo_value = 180 - left_wrist_servo_value

        #finally we need trim to an integer
        left_wrist_servo_value = int(left_wrist_servo_value)


        if bno_head is not None:
            command_bytes[HEAD_YAW] = tracked_head_yaw
            command_bytes[HEAD_PITCH] = tracked_head_pitch
        if bno_left_shoulder is not None:
            command_bytes[LEFT_SHOULDER_YAW] = left_shoulder_yaw_servo_value
            command_bytes[LEFT_SHOULDER_PITCH] = left_shoulder_pitch_servo_value
            command_bytes[LEFT_ELBOW_YAW] = left_elbow_yaw_servo_value
        if bno_left_wrist is not None:
            command_bytes[LEFT_ELBOW_PITCH] = left_elbow_pitch_servo_value
            command_bytes[LEFT_WRIST] = left_wrist_servo_value

        signed_command = bytearray(PACKET_SIGNATURE) + bytearray(command_bytes)
        packet = rfm9x.send(signed_command)
        print(
            speed_command_byte
            )
        time.sleep(0.01)
    except Exception as e:
        log_error(e)
        time.sleep(0.01)