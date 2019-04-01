import rospy
from std_msgs.msg import Int16
import RPi.GPIO as GPIO

def publish_encoder():
  L_pub = rospy.Publisher('lwheel', Int16, queue_size=10)
  R_pub = rospy.Publisher('rwheel', Int16, queue_size=10)
  rospy.init_node('encoder', anonymous=True)
  rate = rospy.Rate(10) # 10hz

  e1_A_pin = 22
  e1_B_pin = 23
  e2_A_pin = 24
  e2_B_pin = 25

  GPIO.setmode(GPIO.BCM)
  GPIO.setup(e1_A_pin, GPIO.IN)
  GPIO.setup(e1_B_pin, GPIO.IN)
  GPIO.setup(e2_A_pin, GPIO.IN)
  GPIO.setup(e2_B_pin, GPIO.IN)

  outcome = [0,-1,1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0]

  e1_last_AB = 0b00
  e2_last_AB = 0b00
  e1 = 0
  e2 = 0

  while not rospy.is_shutdown():
    e1_A = GPIO.input(e1_A_pin)
    e1_B = GPIO.input(e1_B_pin)
    e2_A = GPIO.input(e2_A_pin)
    e2_B = GPIO.input(e2_B_pin)
    e1_curr_AB = (e1_A << 1) | e1_B
    e2_curr_AB = (e2_A << 1) | e2_B
    e1_pos = (e1_last_AB << 2) | e1_curr_AB
    e2_pos = (e2_last_AB << 2) | e2_curr_AB
    e1 += outcome[e1_pos]
    e2 += outcome[e2_pos]
    e1_last_AB = e1_curr_AB
    e2_last_AB = e2_curr_AB
    #print("R {} L {}".format(e1, e2))
    L_pub.publish(e2)
    R_pub.publish(e1)
    rate.sleep()

if __name__ == '__main__':
  try:
    publish_encoder()
  except rospy.ROSInterruptException:
    pass
