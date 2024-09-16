import rclpy
from std_msgs.msg import Bool, Int8, String
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
        self.speech = self.create_subscription(
            String, 'llm/speech/audio',
            self.sendAudio,
            qos_profile=0
        )
        self.audio = self.create_publisher(String, 'microphone', qos_profile=0)
        self.validator = self.create_publisher(Bool, 'llm/speech/audio/poll', qos_profile=0)

    def changeSpeakingState(self, msg):
        self.get_logger().info(f'Speaking: {msg.data}')
        self.mgr.speaking = msg.data

    def callback(self, msg):
        self.get_logger().info(f'New emotion: {msg.data}')
        self.mgr.emotion = msg.data
    
    def sendAudio(self, msg):
        self.get_logger().info('New audio')
        self.mgr.audio = msg.data

def init_node(mgr, args = None):
    rclpy.init(args=args)
    node = HelloRICUI(mgr)
    return node