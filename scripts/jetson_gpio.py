#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import roslib.packages
import time
import math
import Jetson.GPIO as GPIO
import threading
from std_msgs.msg import Bool
from std_msgs.msg import Int32

# Define pins
input_pin = 37  # BOARD pin 37, BCM pin 26 (Linux GPIO 12)
output_pins = [33, 32, 31]  # [GREEN, YELLOW, RED]
nova_station_status = 0
prev_status = -1
blink_thread = None
blink_flag = False

def blink_lights(pin):
    global blink_flag
    while blink_flag:
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(1)  # Blink interval
        GPIO.output(pin, GPIO.LOW)
        time.sleep(1)  # Blink interval

def nova_station_status_callback(msg):
    global prev_status, nova_station_status
    nova_station_status = msg.data

    if prev_status != nova_station_status:
        global blink_thread, blink_flag

        # Turn off the blinking thread if it's active
        if blink_thread:
            blink_flag = False
            blink_thread.join()  # Wait for the thread to finish
            time.sleep(1)

        print("nova_station_status: %d" % nova_station_status)
        if nova_station_status == 0:
            GPIO.output(output_pins[0], GPIO.HIGH)
            GPIO.output(output_pins[1], GPIO.LOW)
            GPIO.output(output_pins[2], GPIO.LOW)
        elif nova_station_status == 1:
            GPIO.output(output_pins[1], GPIO.HIGH)
            blink_flag = True
            blink_thread = threading.Thread(target=blink_lights, args=(output_pins[2],))
            blink_thread.start()
        elif nova_station_status == 2:
            GPIO.output(output_pins[1], GPIO.HIGH)
        elif nova_station_status == 3:
            GPIO.output(output_pins[2], GPIO.HIGH)
            blink_flag = True
            blink_thread = threading.Thread(target=blink_lights, args=(output_pins[2],))
            blink_thread.start()
        elif nova_station_status == 4:
            GPIO.output(output_pins[2], GPIO.HIGH)
        else:
            for pin in output_pins:
                GPIO.output(pin, GPIO.LOW)
        prev_status = nova_station_status

def pub_pin():
    global input_pin
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(input_pin, GPIO.IN)  # set pin as an input pin
    for pin in output_pins:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)  # Initialize to LOW

    prev_value = None
    rospy.init_node('pub_pin')
    pin_pub = rospy.Publisher("GPIO_status", Bool, queue_size=1)
    jetson_status_sub = rospy.Subscriber("nova_station_status", Int32, nova_station_status_callback)
    msg = Bool()
    msg.data = False
    rate = rospy.Rate(100)
    prev_msg = False
    print("Publishing GPIO states now! Press ctrl + c to exit")

    try:
        while not rospy.is_shutdown():
            value = GPIO.input(input_pin)
            if value != prev_value:
                if value == GPIO.HIGH:
                    msg.data = True
                else:
                    msg.data = False
                if msg.data != prev_msg:
                    print("Value read from pin {} : {}".format(input_pin, "HIGH" if msg.data else "LOW"))
                prev_value = value
                prev_msg = msg.data
                time.sleep(2)  # Adjust as needed
                value_after_delay = GPIO.input(input_pin)
                if value_after_delay == value:
                    pin_pub.publish(msg)
            rate.sleep()
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    pub_pin()

