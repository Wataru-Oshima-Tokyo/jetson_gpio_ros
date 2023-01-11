#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import roslib.packages
import time
import RPi.GPIO as GPIO
from std_msgs.msg import Bool


def pub_pin(input_pin):
    prev_value = None
    rospy.init_node('pub_pin')
    pin_pub = rospy.Publisher("input_status", Bool, queue_size=1)
    msg = Bool()
    msg.data = False
    rate = rospy.Rate(100)
    
    prev_value = None
    prev_msg = False
    GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
    GPIO.setup(input_pin, GPIO.IN)  # set pin as an input pin
    print("Starting demo now! Press CTRL+C to exit")
    now = rospy.Time().now()
    try:
        while not rospy.is_shutdown():
            value = GPIO.input(input_pin)
            # print(value)
            if value != prev_value:
                if value == GPIO.HIGH:
                    value_str = "HIGH"
                    now = rospy.Time().now()
                    msg.data = True
                else:
                    value_str = "LOW"
                    msg.data = False
                if msg.data != prev_msg:
                    print("Value read from pin {} : {}".format(input_pin,
                                                         value_str))
                prev_value = value
                prev_msg = msg.data
            if value == GPIO.HIGH and now+rospy.Duration(3) < rospy.Time().now():
                GPIO.setup(input_pin, GPIO.OUT)
                GPIO.output(input_pin, GPIO.LOW)
                GPIO.setup(input_pin, GPIO.IN)
                prev_value = None

            pin_pub.publish(msg)
            rate.sleep()
    finally:
        GPIO.cleanup()

      

if __name__ == '__main__':
  input_pin = 18  # BCM pin 18, BOARD pin 12
  pub_pin(input_pin)