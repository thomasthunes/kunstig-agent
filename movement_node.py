#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Float32MultiArray, Int16MultiArray
import time
import random
from enum import Enum


class MovementStateSubscriber(Node):
    def __init__(self):
        super().__init__("MovementStateSubscriber")
        self.subscription = self.create_subscription(Int8, "BryterTilstandPublisher", self.lytter_callback, 1)
        self.state = 0

    def lytter_callback(self, string_data):
        mottatt = string_data.data
        state = mottatt
        self.set_state(state)

    def set_state(self, state):
        self.state = state


class ObjectPositionSubscriber(Node):
    def __init__(self):
        super().__init__("ObjectPositionSubscriber")
        self.subscription = self.create_subscription(Int16MultiArray, "ObjectPosition", self.lytter_callback, 1)
        self.x = None
        self.w = None

    def lytter_callback(self, string_data):
        mottatt = string_data.data
        pos = mottatt
        self.set_pos(pos)

    def set_pos(self, pos):
        self.x = pos[0]
        self.w = pos[1]


class MovementPublisher(Node):
    def __init__(self):
        super().__init__("MovementPublisher")
        self.publisher = self.create_publisher(Float32MultiArray, "MovementChannel", 1)
        self.left_forward = 1
        self.right_forward = 1
        self.left_backward = 0
        self.right_backward = 0
        self.speed_low = 20
        self.speed_mid = 30
        self.speed_high = 40

    def switch_on_movement(self):
        self.publish(self.left_backward, self.right_forward, 2.35, self.speed_high)

    def object_is_left(self, x):
        if 429 <= x <= 640:
            return True
        return False

    def object_is_center(self, x):
        if 215 <= x <= 428:
            return True
        return False

    def object_is_right(self, x):
        if 0 <= x <= 214:
            return True
        return False

    def turned_too_much(self, x, prev_x):
        if self.object_is_left(x) and self.object_is_right(prev_x):
            return True
        if self.object_is_right(x) and self.object_is_left(prev_x):
            return True
        return False

    def movement_decision(self, x, prev_x):
        duration = 0.5
        print("starting to make a decition")
        if prev_x is None or x is None:
            print(f"prev_x is {prev_x} and x is {x}, returning")
            return
        if self.turned_too_much(x, prev_x):
            duration = 0.25

        if self.object_is_left(x):
            self.publish(self.left_backward, self.right_forward, duration, self.speed_high)
            print(f"Moved left, because x is {x}")
        elif self.object_is_right(x):
            self.publish(self.left_forward, self.right_backward, duration, self.speed_high)
            print(f"Moved right, because x is {x}")
        elif self.object_is_center(x):
            self.publish(self.left_forward, self.right_forward, duration, self.speed_high)
            print(f"Moved forward, because x is {x}")
        else:
            duration = 0.1
            self.publish(self.left_forward, self.right_backward, duration, self.speed_high)
            print("Looking around")

    def publish(self, left, right, duration, speed):
        msg = Float32MultiArray()
        msg.data = [float(left), float(right), float(duration), float(speed)]
        self.publisher.publish(msg)


def main():
    rclpy.init()
    min_subscriber = MovementStateSubscriber()
    object_pos = ObjectPositionSubscriber()

    # Instatiates the publisher
    min_publisher = MovementPublisher()
    try:
        prev_state = 0
        prev_x = None
        while True:
            rclpy.spin_once(min_subscriber, timeout_sec=1)
            rclpy.spin_once(object_pos, timeout_sec=1)
            state = min_subscriber.state

            if prev_state == 0 and state == 1:
                # Publish movement to engine
                min_publisher.switch_on_movement()

            elif prev_state == 1 and state == 1:
                x = object_pos.x
                w = object_pos.w
                if x == 0:
                    x = w // 2
                min_publisher.movement_decision(x, prev_x)
                prev_x = x

            #time.sleep(2)
            prev_state = state
    except KeyboardInterrupt:
        print('\nStoppa av brukaren.')
    finally:
        min_subscriber.destroy_node()
        min_publisher.destroy_node()


if __name__ == '__main__':
    main()
