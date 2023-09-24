#!/usr/bin/env python3

from picamera2 import Picamera2
import rclpy
import cv2
from rclpy.node import Node
from project_sign.msg import ImageData
import numpy as np
import cv2 as cv
import time
from std_msgs.msg import Int16MultiArray


class Bildebehandler(Node):
    def __init__(self):
        super().__init__('bildebehandler_node')
        self.subscription = self.create_subscription(ImageData, 'kamera', self.callback, 1)
        self.image = None

    def callback(self, msg):
        bytes = msg.data
        image = np.ndarray((480, 640, 4), np.uint8, bytes)
        self.image = image

    def get_img(self):
        return self.image


class PositionPublisher(Node):
    def __init__(self):
        super().__init__("PositionPublisher")
        self.publisher = self.create_publisher(Int16MultiArray, "ObjectPosition", 1)

    def publish(self, pos):
        msg = Int16MultiArray()
        msg.data = pos
        self.publisher.publish(msg)
        print(f"Publisher published {pos}")


def find_colors(farge, img):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Red
    if farge == "r":
        lower_hsv = np.array([105, 100, 100])
        upper_hsv = np.array([150, 255, 255])
    # Yellow
    elif farge == "y":
        lower_hsv = np.array([85, 100, 100])
        upper_hsv = np.array([100, 255, 255])
    else:
        pass

    mask = cv2.inRange(img_hsv, lower_hsv, upper_hsv)

    kernel = np.ones((11, 11), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    output = cv2.bitwise_and(img, img, mask=mask)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    x, y, w, h = 0, 0, 0, 0
    if len(contours) != 0:
        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)

    return x, y, w, h


def analyze_pic(img):
    # finn det største røde objektet
    red = find_colors('r', img)
    r_x, r_y, r_w, r_h = red[0], red[1], red[2], red[3]

    # finn det største gule objektet
    yellow = find_colors('y', img)
    y_x, y_y, y_w, y_h = yellow[0], yellow[1], yellow[2], yellow[3]

    if (r_w * r_h) > (y_w * y_h):
        print("RED")
        return [r_x, r_w]
    elif (r_w * r_h) < (y_w * y_h):
        print("YELLOW")
        return [y_x, y_w]
    elif (r_w * r_h) == 0 and (y_w * y_h) == 0:
        print("No objects")
        return -1
    else:
        print("There is equally many objects of each color")
        return -1


def main():
    try:
        rclpy.init()
        subscriber = Bildebehandler()
        publisher = PositionPublisher()
        while rclpy.ok():
            rclpy.spin_once(subscriber, timeout_sec=5)
            img = subscriber.get_img()
            print(isinstance(img, np.ndarray))
            if isinstance(img, np.ndarray):
                pos = analyze_pic(img)
                if pos != -1:
                    publisher.publish(pos)
            # time.sleep(1)
    except KeyboardInterrupt:
        print('\nStoppa av brukaren.')
    finally:
        pass


if __name__ == '__main__':
    main()
