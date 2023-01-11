# !/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import roslib.packages
import time
import RPi.GPIO as GPIO
from std_msgs.msg import Bool


def pub_pin(input_pin):
    prev_value = None
    rospy.init_node('pub_pin')
    pin_pub = rospy.Publisher("insert_result", Bool, queue_size=1)
    msg = Bool()
    msg.data = False
    rate = rospy.Rate(10)
    prev_value = None
    GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
    GPIO.setup(input_pin, GPIO.IN)  # set pin as an input pin
    print("Starting demo now! Press CTRL+C to exit")
    try:
        while not rospy.is_shutdown():
            value = GPIO.input(input_pin)
            if value != prev_value:
                msg.data = not msg.data
                prev_value = value
            pin_pub.publish*(msg)
            rate.sleep()
    finally:
        GPIO.cleanup()



    
        
      

if __name__ == '__main__':
  input_pin = 18  # BCM pin 18, BOARD pin 12
  pub_pin(input_pin)