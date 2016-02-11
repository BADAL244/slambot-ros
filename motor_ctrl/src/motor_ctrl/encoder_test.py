import RPi.GPIO as GPIO
from multiprocessing import Process

class Encoder():
  def __init__(self):
    self.e1_A_pin = 22
    self.e1_B_pin = 23
    self.e2_A_pin = 24
    self.e2_B_pin = 25

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(self.e1_A_pin, GPIO.IN)
    GPIO.setup(self.e1_B_pin, GPIO.IN)
    GPIO.setup(self.e2_A_pin, GPIO.IN)
    GPIO.setup(self.e2_B_pin, GPIO.IN)

    self.outcome = [0,-1,1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0]

    self.e1_last_AB = 0b00
    self.e2_last_AB = 0b00
    self.e1 = 0
    self.e2 = 0

    while(True):
      e1_A = GPIO.input(self.e1_A_pin)
      e1_B = GPIO.input(self.e1_B_pin)
      e2_A = GPIO.input(self.e2_A_pin)
      e2_B = GPIO.input(self.e2_B_pin)
      e1_curr_AB = (e1_A << 1) | e1_B
      e2_curr_AB = (e2_A << 1) | e2_B
      e1_pos = (self.e1_last_AB << 2) | e1_curr_AB
      e2_pos = (self.e2_last_AB << 2) | e2_curr_AB
      e1_diff = self.outcome[e1_pos]
      e2_diff = self.outcome[e2_pos]
      self.e1 += e1_diff
      self.e2 += e2_diff
      self.e1_last_AB = e1_curr_AB
      self.e2_last_AB = e2_curr_AB
      print("R {} L {}".format(self.e1, self.e2))

def main():
  Encoder()

if __name__ == "__main__":
    main()
