# Test ROS 2 node to publish some emotions.
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Int8


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('fake_emotion_publisher')
        self._num = 3
        self.publisher_emo = self.create_publisher(
            Int8, 'llm/speech/emotion', 10)
        self.publisher_speak = self.create_publisher(
            Bool, 'llm/speech/speaking', 10)
        timer_period = 1.0  # publish new emotion every second
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """send emotion 3 and 5 alternating."""
        try:
            msg = Int8()
            self._num = 3 if self._num == 5 else 5
            msg.data = self._num
            self.publisher_emo.publish(msg)
            self.get_logger().info(f'Publish emotion: "{msg.data}"')

            msg = Bool()
            msg.data = self._num == 5
            self.publisher_speak.publish(msg)
            self.get_logger().info(f'Publish speaking: "{msg.data}"')
        except KeyboardInterrupt:
            # we ignore keyboard interrupts as they are send by our
            # testing framework on purpose to shutdown after the test
            # so we don't get tracebacks/unwanted error messages
            pass


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        # we ignore keyboard interrupts as they are send by our
        # testing framework on purpose to shutdown after the test
        # so we don't get tracebacks/unwanted error messages
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()


if __name__ == '__main__':
    main()
