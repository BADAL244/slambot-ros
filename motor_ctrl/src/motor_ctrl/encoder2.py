from gpiozero import DigitalInputDevice
from adafruit_motorkit import MotorKit
from time import sleep

class QuadratureEncoder(object):
    """
    A simple quadrature encoder class
    Note - this class does not determine direction
    """
    def __init__(self, pin_a, pin_b):
        self._value = 0

        encoder_a = DigitalInputDevice(pin_a)
        encoder_a.when_activated = self._increment
        encoder_a.when_deactivated = self._increment

        encoder_b = DigitalInputDevice(pin_b)
        encoder_b.when_activated = self._increment
        encoder_b.when_deactivated = self._increment
        
    def reset(self):
        self._value = 0

    def _increment(self):
        self._value += 1

    @property
    def value(self):
        return self._value

kit = MotorKit()

e1 = QuadratureEncoder(27, 22)
e2 = QuadratureEncoder(23, 24)

#start the robot
kit.motor1.throttle = None
kit.motor2.throttle = None

while True:
    print("e1 {} e2 {}".format(e1.value, e2.value))
    sleep(1)
