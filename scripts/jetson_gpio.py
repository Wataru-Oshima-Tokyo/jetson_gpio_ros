
#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import roslib.packages
import time
import math
import Jetson.GPIO as GPIO
from std_msgs.msg import Bool
from std_msgs.msg import Int32
# define pins
input_pin = 37 # BOARD pin 37, BCM pin 26 (Linux GPIO 12)
output_pins = [33, 32, 31] # [GREEN, YELLOW, RED]
nova_station_status = 0
prev_status = -1

blink_thread = None
blink_flag = Falsey

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
    """
    0 - boot up /wait
    1 - got the start signal and moving 
    2 - finished the action and charging
    3 - failed to insert
    4 - failed all the sequence / something wrong(error)
    """
    GPIO.setmode(GPIO.BOARD) # the pin number of the 40 pin GPIO header from Jetson-nano
    GPIO.setup(output_pin, GPIO.OUT)
    prev_value = None
    if prev_status != nova_station_status:
        global blink_thread, blink_flag

        # Turn off the blinking thread if it's active
        if blink_thread:
            blink_flag = False
            blink_thread.join()  # Wait for the thread to finish
            time.sleep(1)

        print("nova_station_status: %d" % nova_station_status)
        if nova_station_status == 0:
            GPIO.output(channel, (GPIO.HIGH, GPIO.LOW, GPIO.LOW))
        elif nova_station_status == 1:
            GPIO.output(channel, (GPIO.LOW, GPIO.HIGH, GPIO.LOW))
        elif nova_station_status == 2:
            GPIO.output(channel, (GPIO.LOW, GPIO.HIGH, GPIO.LOW))
        elif nova_station_status == 3:
            GPIO.output(channel, (GPIO.LOW, GPIO.LOW, GPIO.HIGH))
        elif nova_station_status == 4:
            GPIO.output(channel, (GPIO.LOW, GPIO.LOW, GPIO.HIGH))
        else:
            ROS_WARN_STREAM("Invalid situation: " << nova_station_status);
    prev_status = nova_station_status    

def pub_pin():
    global input_pin
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
        GPIO.cleanup(input_pin)


if __name__ == '__main__':
  pub_pin()