#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
import time


class EngineSubscriber(Node):
    def __init__(self):
        super().__init__("EngineSubscriber")
        self.right = None
        self.left = None
        self.duration = None
        self.speed = None
        self.subscription = self.create_subscription(Float32MultiArray, "MovementChannel", self.lytter_callback, 1)

    def lytter_callback(self, string_data):
        mottatt = string_data.data
        instructions = mottatt
        self.set_left(instructions[0])
        self.set_right(instructions[1])
        self.set_duration(instructions[2])
        self.set_speed(instructions[3])


    def set_duration(self, duration):
        self.duration = duration

    def set_left(self, left):
        self.left = left

    def set_right(self, right):
        self.right = right

    def set_speed(self, speed):
        self.speed = speed



class InstructEngine():
    def __init__(self):
        self.movement_left = {
            1: 5,
            0: 6
        }
        self.movement_right = {
            1: 13,
            0: 19
        }


    def setup(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(5, GPIO.OUT)
        GPIO.setup(6, GPIO.OUT)
        GPIO.setup(13, GPIO.OUT)
        GPIO.setup(19, GPIO.OUT)


    def move(self, left, right, duration, speed):
        pwm_v = GPIO.PWM(self.movement_left[left], 100)
        pwm_h = GPIO.PWM(self.movement_right[right], 100)

        pwm_v.start(speed+2)
        pwm_h.start(speed)
        time.sleep(duration)

        pwm_v.stop()
        pwm_h.stop()


def main():
    rclpy.init()
    min_subscriber = EngineSubscriber()
    engine = InstructEngine()
    engine.setup()
    try:
        while True:
            rclpy.spin_once(min_subscriber, timeout_sec=5)
            #engine = InstructEngine()
            #engine.setup()

            left = min_subscriber.left
            right = min_subscriber.right
            duration = min_subscriber.duration
            speed = min_subscriber.speed
            if left != None and right != None and duration != None and speed != None:
                engine.move(left, right, duration, speed)
                time.sleep(1)
    except KeyboardInterrupt:
        print('\nStoppa av brukaren.')
    finally:
        min_subscriber.destroy_node()


if __name__ == '__main__':
    main()
