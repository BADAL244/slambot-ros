import rospy
import time

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry

from adafruit_motorkit import MotorKit

class MotorController:
  def __init__(self):
    rospy.init_node("motor")
    self.nodename = rospy.get_name()
    rospy.loginfo("%s started" % self.nodename)
    self.mk = MotorKit()
    rospy.on_shutdown(self.motor_off)
    rospy.Subscriber("motor/left/motor_cmd", Float32, callback=self.on_left)
    rospy.Subscriber("motor/right/motor_cmd", Float32, callback=self.on_right)

  def on_left(self, motor_cmd):
    mk.motor1.throttle = motor_cmd

  def on_right(self, motor_cmd):
    mk.motor2.throttle = motor_cmd

  def motor_off():
    mk.motor1.throttle = 0
    mk.motor2.throttle = 0

def main():
  MotorController()
  rospy.spin()