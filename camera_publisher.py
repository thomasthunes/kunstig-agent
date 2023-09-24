#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from project_sign.msg import ImageData
from picamera2 import Picamera2
import cv2 as cv
import numpy as np
import time


class KameraNode(Node):
    def __init__(self):
        super().__init__('kamera_node')
        self.camera = Picamera2()
        self.camera.start()
        self.publisher = self.create_publisher(ImageData, 'kamera', 1)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        img = self.camera.capture_array()

        msg = ImageData()
        msg.data = img.tobytes()
        self.publisher.publish(msg)


def main():
    try:
        rclpy.init()
        publisher = KameraNode()

        while rclpy.ok():
            rclpy.spin(publisher)
            time.sleep(7)
    except KeyboardInterrupt:
        print('\nStoppa av brukaren.')
    finally:
        pass


if __name__ == '__main__':
    main()
