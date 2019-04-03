import rospy
import time
import atexit

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

class MotorController:
  def __init__(self):
    rospy.init_node("motor")
    self.nodename = rospy.get_name()
    rospy.loginfo("%s started" % self.nodename)
    mh = Adafruit_MotorHAT(addr=0x60)
    self.left = mh.getMotor(1)
    self.right = mh.getMotor(2)
    rospy.on_shutdown(self.motor_off)
    atexit.register(self.motor_off)
    rospy.Subscriber("left/motor_cmd", Float32, callback=self.on_left)
    rospy.Subscriber("right/motor_cmd", Float32, callback=self.on_right)

  def on_left(self, motor_cmd):
    motor_cmd = int(motor_cmd.data)
    print("left_cmd: %d") % (motor_cmd)
    if motor_cmd == 0:
      print("Release Left")
      self.left.run(Adafruit_MotorHAT.RELEASE)
    elif motor_cmd > 0:
      self.left.setSpeed(motor_cmd)
      self.left.run(Adafruit_MotorHAT.FORWARD)
    else:
      self.left.setSpeed(-motor_cmd)
      self.left.run(Adafruit_MotorHAT.BACKWARD)

  def on_right(self, motor_cmd):
    motor_cmd = int(motor_cmd.data)
    print("right_cmd: %d") % (motor_cmd)
    if motor_cmd == 0:
      print("Release Right")
      self.right.run(Adafruit_MotorHAT.RELEASE)
    elif motor_cmd > 0:
      self.right.setSpeed(motor_cmd)
      self.right.run(Adafruit_MotorHAT.FORWARD)
    else:
      self.right.setSpeed(-motor_cmd)
      self.right.run(Adafruit_MotorHAT.BACKWARD)

  def motor_off(self):
    self.left.run(Adafruit_MotorHAT.RELEASE)
    self.right.run(Adafruit_MotorHAT.RELEASE)

def main():
  MotorController()
  rospy.spin()