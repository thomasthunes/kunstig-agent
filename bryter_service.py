#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import String
import time
import random
from project_sign.srv import ProjectBrytertilstand


class TjenesteNode(Node):
    def __init__(self):
        super().__init__('tjeneste_node')
        self.srv = self.create_service(ProjectBrytertilstand, 'BryterTilstand', self.handle_ProjectBrytertilstand)

    def inspect_switch(self):
        GPIO.setwarnings(False)

        GPIO.setmode(GPIO.BCM)
        kobling_inn = 4
        kobling_ut = 17
        GPIO.setup(kobling_inn, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(kobling_ut, GPIO.OUT)
        tilstand_inn = GPIO.input(kobling_inn)
        GPIO.output(kobling_ut, GPIO.HIGH)
        if tilstand_inn == 1:
            # print("ON")
            return 1
        else:
            # print("OFF")
            return 0

    def handle_ProjectBrytertilstand(self, request, response):
        request.state = self.inspect_switch()
        response.result = request.state
        return response


def main():
    rclpy.init()
    min_instans = TjenesteNode()
    try:
        while True:
            rclpy.spin(min_instans)
            time.sleep(1)
    except KeyboardInterrupt:
        print('KeyboardInterrupt exception is caught')
    finally:
        min_instans.destroy_node()
        # rclpy.shutdown()


if __name__ == '__main__':
    main()
