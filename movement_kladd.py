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

    def object_is_right(self, x):
        if 215 <= x <= 428:
            return True
        return False

    def object_is_center(self, x):
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
        duration = 1
        if prev_x is None:
            pass
        if self.turned_too_much(x, prev_x):
            duration = 0.5

        if self.object_is_left(x):
            self.publish(self.left_backward, self.right_forward, duration, self.speed_mid)
        elif self.object_is_right(x):
            self.publish(self.left_forward, self.right_backward, duration, self.speed_mid)
        elif self.object_is_center(x):
            self.publish(self.left_forward, self.right_forward, duration, self.speed_mid)
        else:
            self.publish(self.left_forward, self.right_backward, duration, self.speed_low)

    def publish(self, left, right, duration, speed):
        msg = Float32MultiArray()
        msg.data = [float(left), float(right), float(duration), float(speed)]
        self.publisher.publish(msg)
