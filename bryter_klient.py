#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from project_sign.srv import ProjectBrytertilstand
from time import sleep
from std_msgs.msg import Int8


class KlientNode(Node):
    def __init__(self):
        super().__init__('klient_node')
        self.cli = self.create_client(ProjectBrytertilstand, 'BryterTilstand')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            print('Tjeneste utilgjengelig, venter...')

    def send_request(self):
        request = ProjectBrytertilstand.Request()

        self.future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        result = self.future.result()
        state = result.result
        print(state)
        self.set_state(state)


    def set_state(self, state):
        self.state = state


class Publisher(Node):
    def __init__(self):
        super().__init__("BryterTilstandPublisher")
        self.publisher = self.create_publisher(Int8, "BryterTilstandPublisher", 10)

    def publish(self, state):
        msg = Int8()
        msg.data = state
        self.publisher.publish(msg)


def main():
    try:
        rclpy.init()
        klient = KlientNode()
        publisher = Publisher()
        while rclpy.ok():
            klient.send_request()
            state = klient.state
            publisher.publish(state)
            sleep(1)
    except KeyboardInterrupt:
        print('\nStoppa av brukaren.')
    finally:
        pass


if __name__ == '__main__':
    main()




