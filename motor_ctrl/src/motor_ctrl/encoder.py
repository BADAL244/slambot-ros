from gpiozero import DigitalInputDevice
from adafruit_motorkit import MotorKit
from time import sleep
import atexit

kit = MotorKit()

class Encoder(object):
    def __init__(self, pin):
        self._value = 0

        # setup gpiozero to call increment on each when_activated
        encoder = DigitalInputDevice(pin)
        encoder.when_activated = self._increment
        encoder.when_deactivated = self._increment
        
    def reset(self):
        self._value = 0

    def _increment(self):
        self._value += 1

    @property
    def value(self):
        return self._value

def motorOff():
    kit.motor1.throttle = None
    kit.motor2.throttle = None

SAMPLETIME = 1

e1 = Encoder(27)
e2 = Encoder(22)
e3 = Encoder(23)
e4 = Encoder(24)

#start the robot
kit.motor1.throttle = None
kit.motor2.throttle = None

#find a sample rate
while True:
#    kit.motor2.throttle = 0.2
    print("e1 {} e2 {}".format(e1.value, e2.value))
    sleep(SAMPLETIME)

atexit.register(motorOff)
