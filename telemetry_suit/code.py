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
bno_chest = BNO08X_I2C(tca[0])
bno_chest.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
bno_head = BNO08X_I2C(tca[7], address=0x4B)
bno_head.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
bno_left_shoulder = BNO08X_I2C(tca[1])
bno_left_shoulder.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
bno_left_wrist = BNO08X_I2C(tca[1], address=0x4B)
bno_left_wrist.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
#initialize rfm95
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.A5)
reset = digitalio.DigitalInOut(board.A4)
import adafruit_rfm9x
rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, 915.0)

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
tracked_body.joint_chest.bno085 = bno_chest
tracked_body.joint_head.bno085 = bno_head
tracked_body.joint_left_shoulder.bno085 = bno_left_shoulder
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
        left_shoulder_raw_euler =  Quaternion.to_euler(tracked_body.joint_left_shoulder.offset_orientation)
        left_shoulder_yaw_euler =  Quaternion.to_euler(tracked_body.joint_left_shoulder.servo_orientation)
        left_wrist_euler = Quaternion.to_euler(tracked_body.joint_left_wrist.servo_orientation)
        #inverse left wrist yaw
        left_wrist_euler.yaw = EulerOrientation.inverse180(EulerOrientation.get_180_servo_angle(EulerOrientation.inverse360(left_wrist_euler.yaw)))
        left_wrist_euler.roll = EulerOrientation.inverse180(EulerOrientation.get_180_servo_angle(EulerOrientation.add_angles(90, left_wrist_euler.roll)))
        #the neutral head pitch and yaw on ALICE's head is both 90, but it's 0 for the joints,
        #so we need to add 90 degrees to each of these angles

        tracked_head_yaw = int(EulerOrientation.add_angles(90, head_yaw_euler.yaw))
        tracked_head_pitch = int(EulerOrientation.add_angles(90, head_pitch_euler.pitch))
        tracked_left_shoulder_yaw = int(EulerOrientation.add_angles(90, left_shoulder_yaw_euler.yaw))
        tracked_left_shoulder_pitch = EulerOrientation.inverse180(
            int(EulerOrientation.add_angles(90, left_shoulder_raw_euler.pitch))
            )
        tracked_left_shoulder_roll = EulerOrientation.inverse180(
                int(EulerOrientation.add_angles(90, left_shoulder_raw_euler.roll))
            )

        tracked_head_yaw = EulerOrientation.get_180_servo_angle(tracked_head_yaw)
        tracked_head_pitch = EulerOrientation.get_180_servo_angle(tracked_head_pitch)
        tracked_left_shoulder_yaw = EulerOrientation.get_180_servo_angle(tracked_left_shoulder_yaw)
        LEFT_SHOULDER_PITCH_MIN = 35
        LEFT_SHOULDER_PITCH_MAX = 165
        tracked_left_shoulder_pitch = int(EulerOrientation.get_180_servo_angle(
            (
                (tracked_left_shoulder_pitch-LEFT_SHOULDER_PITCH_MIN)
                / (LEFT_SHOULDER_PITCH_MAX-LEFT_SHOULDER_PITCH_MIN)
                ) * 180
            )
        )
        LEFT_SHOULDER_ROLL_MIN = 40
        LEFT_SHOULDER_ROLL_MAX = 100
        tracked_left_shoulder_roll = int(EulerOrientation.get_180_servo_angle(
            (
                (tracked_left_shoulder_roll-LEFT_SHOULDER_ROLL_MIN)
                / (LEFT_SHOULDER_ROLL_MAX-LEFT_SHOULDER_ROLL_MIN)
                ) * 90
            )
        ) + 90
        if tracked_left_shoulder_roll >180:tracked_left_shoulder_roll=180

        tracked_left_wrist_yaw = int(left_wrist_euler.yaw)
        if tracked_left_wrist_yaw < 90:
            if tracked_left_wrist_yaw > 45 : tracked_left_wrist_yaw = 90
            else: tracked_left_wrist_yaw = 180


        #command_bytes[HEAD_YAW] = tracked_head_yaw
        #command_bytes[HEAD_PITCH] = tracked_head_pitch
        command_bytes[LEFT_SHOULDER_YAW] = tracked_left_shoulder_yaw
        command_bytes[LEFT_SHOULDER_PITCH] = tracked_left_shoulder_pitch
        command_bytes[LEFT_ELBOW_YAW] = tracked_left_shoulder_roll

        command_bytes[LEFT_ELBOW_PITCH] = int(left_wrist_euler.yaw)
        #command_bytes[LEFT_WRIST] = int(left_wrist_euler.roll)

        signed_command = bytearray(PACKET_SIGNATURE) + bytearray(command_bytes)
        packet = rfm9x.send(signed_command)
        print(
            "elbow pitch: " + str(tracked_left_wrist_yaw)
            # "pitch: "
#             + str(int(tracked_left_shoulder_pitch))
#             + "\t\tyaw: " + str(int(tracked_left_shoulder_yaw))
#             + "\t\troll: " + str(int(tracked_left_shoulder_roll))
            )
        #time.sleep(0.1)
    except Exception as e:
        log_error(e)
        time.sleep(0.1)