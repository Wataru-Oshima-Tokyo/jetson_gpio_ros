#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import roslib.packages
import time
import Jetson.GPIO as GPIO
from std_msgs.msg import Bool

# define pins
input_pin = 37 # BOARD pin 37, BCM pin 26 (Linux GPIO 12)

def pub_pin():
    GPIO.setmode(GPIO.BOARD) # the pin number of the 40 pin GPIO header from Jetson-nano
    prev_value = None
    rospy.init_node('pub_pin')
    pin_pub = rospy.Publisher("GPIO_status", Bool, queue_size=1)
    msg = Bool()
    msg.data = False
    rate = rospy.Rate(100)
    prev_msg = False
    GPIO.setup(input_pin, GPIO.IN)  # set pin as an input pin
    print("Publishing GPIO states now! Press ctrl + c to exit")
    now = rospy.Time().now()
    try:
        while not rospy.is_shutdown():
            value = GPIO.input(input_pin)
            # print(value)
            if value != prev_value:
                now = rospy.Time().now()
                if value == GPIO.HIGH:
                    value_str = "HIGH"
                else:
                    value_str = "LOW"
                print("Value read from pin {} : {}".format(input_pin,
                                                         value_str))  
                prev_value = value
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
  pub_pin()