import rclpy
import requests
from std_msgs.msg import Int8, Bool
from rclpy.node import Node


NODE_NAME = 'HelloRIC_ui_com'


class HelloRICUI(Node):
    def __init__(self, mgr):
        super().__init__('HelloRIC_ui_com')
        self.mgr = mgr
        self.speak = self.create_subscription(
            Bool, 'llm/speech/speaking',
            self.changeSpeakingState,
            qos_profile = 0)
        self.sub = self.create_subscription(
            Int8, 'llm/speech/emotion',
            self.callback,
            qos_profile = 0)

    def changeSpeakingState(self, msg):
        self.get_logger().info(f'Speaking: {msg.data}')
        self.mgr.speaking = msg.data

    def callback(self, msg):
        self.get_logger().info(f'New emotion: {msg.data}')
        self.mgr.emotion = msg.data

def init_node(mgr, args = None):
    rclpy.init(args=args)
    node = HelloRICUI(mgr)
    return node