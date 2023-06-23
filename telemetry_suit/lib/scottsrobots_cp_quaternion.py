import math

class EulerOrientation:
    def __init__(self, roll=0,pitch=0,yaw=0):
        self.roll, self.pitch, self.yaw = roll,pitch,yaw

class Quaternion:
    #static helper methods
    @staticmethod
    def from_quaternion(q):
        return Quaternion(q.qx,q.qy,q.qz,q.qw)

    @staticmethod
    def from_tuple(t):
        #assumes format qx,qy,qz,qw
        qx,qy,qz,qw = t
        return Quaternion(qx,qy,qz,qw)

    @staticmethod
    def to_euler(q1):
        qx,qy,qz,qw = q1.qx,q1.qy,q1.qz,q1.qw
        roll  = math.atan2(2.0 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
        pitch = -math.asin(2.0 * (qx * qz - qw * qy))
        yaw   = math.atan2(2.0 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz)
        pitch *= 180.0 / math.pi
        yaw   *= 180.0 / math.pi
        if yaw < 0: yaw   += 360.0  # Ensure yaw stays between 0 and 360
        roll  *= 180.0 / math.pi
        return EulerOrientation(roll=roll,pitch=pitch,yaw=yaw)

    @staticmethod
    def multiply(q1,q2):
        #adapted from https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/code/index.htm
        return Quaternion(
            qx = q1.qx * q2.qw + q1.qy * q2.qz - q1.qz * q2.qy + q1.qw * q2.qx
            ,qy = -q1.qx * q2.qz + q1.qy * q2.qw + q1.qz * q2.qx + q1.qw * q2.qy
            ,qz = q1.qx * q2.qy - q1.qy * q2.qx + q1.qz * q2.qw + q1.qw * q2.qz
            ,qw = -q1.qx * q2.qx - q1.qy * q2.qy - q1.qz * q2.qz + q1.qw * q2.qw
        )

    @staticmethod
    def conjugate(q1): #conjugate = inverse
        #adapted from https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/code/index.htm
        return Quaternion(
            qx = -q1.qx
            ,qy = -q1.qy
            ,qz = -q1.qz
            ,qw = q1.qw
        )

    #instance methods
    def __init__(self,qx=0,qy=0,qz=0,qw=0):
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw