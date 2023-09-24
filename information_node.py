#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import time
import random
import RPi.GPIO as GPIO


class InformationSubscriber(Node):
    def __init__(self):
        super().__init__("InformationSubscriber_mef015")
        self.subscription = self.create_subscription(Int8, "BryterTilstandPublisher", self.lytter_callback, 10)

    def lytter_callback(self, string_data):
        mottatt = string_data.data
        state = mottatt
        self.set_state(state)

    def set_state(self, state):
        self.state = state


class CommunicateSignals:

    def __init__(self):
        self.led_pin = 22
        self.dot_duration = 0.5
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.led_pin, GPIO.OUT)

    def led_on(self):
        GPIO.output(self.led_pin, GPIO.HIGH)
        time.sleep(self.dot_duration)
        GPIO.output(self.led_pin, GPIO.LOW)
        time.sleep(self.dot_duration)

        #
        # time.sleep(self.dot_duration)
        #
        # GPIO.output(self.led_pin, GPIO.LOW)
        # GPIO.cleanup()
        #
        # # PÅ
        # GPIO.output(self.led_pin, GPIO.HIGH)
        # time.sleep(self.dot_duration)
        # GPIO.output(self.led_pin, GPIO.LOW)

    def communication_logic(self, state):
        if state == 0:
            n_blinks = 2
        elif state == 1:
            n_blinks = 1
        else:
            # if this happens, something went wrong
            n_blinks = 10

        for i in range(n_blinks):
            self.led_on()
            # 8 milliseconds between each related sequence
            time.sleep(0.8)
        # al least 2 seconds between each message
        # time.sleep(2)


def main():
    rclpy.init()
    min_subscriber = InformationSubscriber()
    min_communicator = CommunicateSignals()

    try:
        last_state = 0
        while True:
            rclpy.spin_once(min_subscriber, timeout_sec=10)
            state = min_subscriber.state
            if last_state != state:
                min_communicator.communication_logic(state)
                last_state = state
            time.sleep(1)
    except KeyboardInterrupt:
        print('\nStoppa av brukaren.')
    finally:
        min_subscriber.destroy_node()


if __name__ == '__main__':
    main()
