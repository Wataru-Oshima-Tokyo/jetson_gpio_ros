#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import roslib.packages
import time
import math
import Jetson.GPIO as GPIO
from std_msgs.msg import Bool, Int32

# define pins
input_pin = 37 # BOARD pin 37, BCM pin 26 (Linux GPIO 12)

def nova_station_status_callback(msg):
    nova_station_status = msg.data
    print("nova_station_status:", nova_station_status)



def pub_pin():
    GPIO.setmode(GPIO.BOARD) # the pin number of the 40 pin GPIO header from Jetson-nano
    prev_value = None
    rospy.init_node('pub_pin')
    pin_pub = rospy.Publisher("GPIO_status", Bool, queue_size=1)
    jetson_status_sub = rospy.Subscribe("nova_station_status", Int32, nova_station_status_callback)
    msg = Bool()
    msg.data = False
    rate = rospy.Rate(100)
    prev_msg = False
    GPIO.setup(input_pin, GPIO.IN)  # set pin as an input pin
    print("Publishing GPIO states now! Press ctrl + c to exit")
    now = rospy.Time().now()
    stable_duration = rospy.Duration(3)  # Adjust this duration as needed
    stable_time = None

    try:
        while not rospy.is_shutdown():
            value = GPIO.input(input_pin)
            # print(value)
            if value != prev_value:
                if value == GPIO.HIGH:
                    value_str = "HIGH"
                    msg.data = True
                else:
                    value_str = "LOW"
                    msg.data = False
                if msg.data != prev_msg:
                    print("Value read from pin {} : {}".format(input_pin,
                                                            value_str))  
                prev_value = value
                prev_msg = msg.data


                time.sleep(2)  # For getting results after voltages become stable, adjust the delay as needed
                value_after_delay = GPIO.input(input_pin)
                if value_after_delay == value:
                    pin_pub.publish(msg)


            rate.sleep()
    finally:
        GPIO.cleanup()


if __name__ == '__main__':
  pub_pin()