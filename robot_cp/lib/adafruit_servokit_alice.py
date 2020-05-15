# The MIT License (MIT)
#
# Copyright (c) 2018 Kattni Rembor for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`adafruit_servokit`
====================================================
CircuitPython helper library for the PWM/Servo FeatherWing, Shield and Pi HAT and Bonnet kits.
* Author(s): Kattni Rembor
Implementation Notes
--------------------
**Hardware:**
* `8-Channel PWM or Servo FeatherWing <https://www.adafruit.com/product/2928>`_
* `Adafruit 16-Channel 12-bit PWM/Servo Shield <https://www.adafruit.com/product/1411>`_
* `Adafruit 16-Channel PWM/Servo HAT for Raspberry Pi <https://www.adafruit.com/product/2327>`_
* `Adafruit 16-Channel PWM/Servo Bonnet for Raspberry Pi <https://www.adafruit.com/product/3416>`_
**Software and Dependencies:**
* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
* Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
* Adafruit's PCA9685 library: https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
* Adafruit's Motor library: https://github.com/adafruit/Adafruit_CircuitPython_Motor
"""

import board
from adafruit_pca9685 import PCA9685
try:
    import struct
except ImportError:
    import ustruct as struct

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_ServoKit.git"
class PWMChannel:
    """A single PCA9685 channel that matches the :py:class:`~pulseio.PWMOut` API."""
    def __init__(self,pca,index):
        self._pca = pca
        self._index = index
        self._duty_cycle = (0,0x1000)
		
    @property
    def frequency(self):
        """The overall PWM frequency in Hertz (read-only).
        A PWMChannel's frequency cannot be set individually.
        All channels share a common frequency, set by PCA9685.frequency."""
        return self._pca.frequency

    @frequency.setter
    def frequency(self, _):
        raise NotImplementedError("frequency cannot be set on individual channels")

    @property
    def duty_cycle(self):
        """16 bit value that dictates how much of one cycle is high (1) versus low (0). 0xffff will
           always be high, 0 will always be low and 0x7fff will be half high and then half low."""
        #return self._duty_cycle
        pwm = self._duty_cycle
        if pwm[0] == 0x1000:
            return 0xffff
        return pwm[1] << 4
    
    @duty_cycle.setter
    def duty_cycle(self, value):
        if not 0 <= value <= 0xffff:
            raise ValueError("Out of range")
        #self._duty_cycle = value
        if value == 0xffff:
            self._duty_cycle = (0x1000, 0)
        else:
            # Shift our value by four because the PCA9685 is only 12 bits but our value is 16
            value = (value + 1) >> 4
            self._duty_cycle = (0, value)

class PCAChannels: # pylint: disable=too-few-public-methods
    """Lazily creates and caches channel objects as needed. Treat it like a sequence."""
    def __init__(self, pca):
        self._pca = pca
        self._channels = [None] * len(self)

    def __len__(self):
        return 16

    def __getitem__(self, index):
        if not self._channels[index]:
            self._channels[index] = PWMChannel(self._pca, index)
        return self._channels[index]
    def get_duty_cycles(self):       
        duty_cycles = [None] * len(self)
        for i, channel in enumerate(self):
            duty_cycles[i] = channel._duty_cycle
        return duty_cycles
        
    
class ServoKit:
    """Class representing an Adafruit PWM/Servo FeatherWing, Shield or Pi HAT and Bonnet kits.
       Automatically uses the I2C bus on a Feather, Metro or Raspberry Pi.
       Initialise the PCA9685 chip at ``address``.
       The internal reference clock is 25MHz but may vary slightly with environmental conditions and
       manufacturing variances. Providing a more precise ``reference_clock_speed`` can improve the
       accuracy of the frequency and duty_cycle computations. See the ``calibration.py`` example in
       the `PCA9685 GitHub repo <https://github.com/adafruit/Adafruit_CircuitPython_PCA9685>`_ for
       how to derive this value by measuring the resulting pulse widths.
       :param int channels: The number of servo channels available. Must be 8 or 16. The FeatherWing
                            has 8 channels. The Shield, HAT, and Bonnet have 16 channels.
       :param int address: The I2C address of the PCA9685. Default address is ``0x40``.
       :param int reference_clock_speed: The frequency of the internal reference clock in Hertz.
                                         Default reference clock speed is ``25000000``.
    """
    def __init__(self, *, channels, i2c=None, address=0x40, reference_clock_speed=25000000):
        if channels not in [8, 16]:
            raise ValueError("servo_channels must be 8 or 16!")
        self._items = [None] * channels
        self._channels = channels
        if i2c is None:
            i2c = board.I2C()
        self._pca = PCA9685(i2c, address=address, reference_clock_speed=reference_clock_speed)
        self._pca.frequency = 50

        self._servo = _Servo(self)
        self._continuous_servo = _ContinuousServo(self)
        self._pcaChannels = PCAChannels(self._pca)

    def setServos(self):
        #get the buffer
        first_register = 0x06 #first LED/Servo register of the PCA
        struct_format = '<HH'
        buf = bytearray(1) #one byte for the initial register address followed by PCA's 16 4-byte registers 
        buf[0] = first_register
        duty_cycles = self._pcaChannels.get_duty_cycles()
        for duty_cycle in duty_cycles:
            pwm_reg = bytearray(struct.calcsize(struct_format)) 
            struct.pack_into(struct_format, pwm_reg, 0, *duty_cycle)
            buf = buf + pwm_reg
        with self._pca.i2c_device:
            self._pca.i2c_device.write(buf)
        
    @property
    def servo(self):
        """:py:class:``~adafruit_motor.servo.Servo`` controls for standard servos.
        This FeatherWing example rotates a servo on channel ``0`` to ``180`` degrees for one second,
        and then returns it to ``0`` degrees.
        .. code-block:: python
            import time
            from adafruit_servokit import ServoKit
            kit = ServoKit(channels=8)
            kit.servo[0].angle = 180
            time.sleep(1)
            kit.servo[0].angle = 0
        """
        return self._servo

    @property
    def continuous_servo(self):
        """:py:class:``~adafruit_motor.servo.ContinuousServo`` controls for continuous rotation
        servos.
        This FeatherWing example rotates a continuous rotation servo on channel ``0`` forward for
        one second, then backward for one second, and then stops the rotation.
        .. code-block:: python
            import time
            from adafruit_servokit import ServoKit
            kit = ServoKit(channels=8)
            kit.continuous_servo[0].throttle = 1
            time.sleep(1)
            kit.continuous_servo[0].throttle = -1
            time.sleep(1)
            kit.continuous_servo[0].throttle = 0
        """
        return self._continuous_servo


class _Servo:
    # pylint: disable=protected-access
    def __init__(self, kit):
        self.kit = kit

    def __getitem__(self, servo_channel):
        import adafruit_motor.servo
        num_channels = self.kit._channels
        if servo_channel >= num_channels or servo_channel < 0:
            raise ValueError("servo must be 0-{}!".format(num_channels - 1))
        servo = self.kit._items[servo_channel]
        if servo is None:
            servo = adafruit_motor.servo.Servo(self.kit._pcaChannels[servo_channel],min_pulse=750,max_pulse=2500)
            #servo = adafruit_motor.servo.Servo(self.kit._pca.channels[servo_channel],min_pulse=750,max_pulse=2500)
            self.kit._items[servo_channel] = servo
            return servo
        if isinstance(self.kit._items[servo_channel], adafruit_motor.servo.Servo):
            return servo
        raise ValueError("Channel {} is already in use.".format(servo_channel))

    def __len__(self):
        return len(self.kit._items)


class _ContinuousServo:
    # pylint: disable=protected-access
    def __init__(self, kit):
        self.kit = kit

    def __getitem__(self, servo_channel):
        import adafruit_motor.servo
        num_channels = self.kit._channels
        if servo_channel >= num_channels or servo_channel < 0:
            raise ValueError("servo must be 0-{}!".format(num_channels - 1))
        servo = self.kit._items[servo_channel]
        if servo is None:
            servo = adafruit_motor.servo.ContinuousServo(self.kit._pca.channels[servo_channel])
            self.kit._items[servo_channel] = servo
            return servo
        if isinstance(self.kit._items[servo_channel], adafruit_motor.servo.ContinuousServo):
            return servo
        raise ValueError("Channel {} is already in use.".format(servo_channel))

    def __len__(self):
        return len(self.kit._items)