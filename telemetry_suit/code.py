import time
import board
import busio
import math
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c)

bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

class EulerOrientation:
    def __init__(self, roll=0,pitch=0,yaw=0):
        self.roll, self.pitch, self.yaw = roll,pitch,yaw

def quaternionToEuler(qw,qx,qy,qz):
    roll  = math.atan2(2.0 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
    pitch = -math.asin(2.0 * (qx * qz - qw * qy))
    yaw   = math.atan2(2.0 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz)
    pitch *= 180.0 / math.pi
    yaw   *= 180.0 / math.pi
    if yaw < 0: yaw   += 360.0  # Ensure yaw stays between 0 and 360
    roll  *= 180.0 / math.pi
    return EulerOrientation(roll=roll,pitch=pitch,yaw=yaw)

while True:
    time.sleep(0.1)
    try:
        #print("Rotation Vector Quaternion:")
        qx, qy, qz, qw = bno.quaternion  # pylint:disable=no-member
        eulerOrientation = quaternionToEuler(qw=qw,qx=qx,qy=qy,qz=qz)
        print("R: %0.6f  P: %0.6f Y: %0.6f" % (eulerOrientation.roll, eulerOrientation.pitch, eulerOrientation.yaw))
    except Exception as e:
        print(e)


    #     print(
    #         "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real)
    #     )
    #     print("")