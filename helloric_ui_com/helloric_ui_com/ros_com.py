import rclpy
from rclpy.node import Node

from ric_messages.msg import PlayAudio

from std_msgs.msg import String


NODE_NAME = 'HelloRIC_ui_com'


class ROSCom(Node):
    def __init__(self, mgr):
        super().__init__('HelloRIC_ui_com')
        self.mgr = mgr
        self.audio_receiver = self.create_subscription(
            PlayAudio, 'websocket/play_audio', self.receiveQueue, 0)
        self.audio = self.create_publisher(String, 'microphone', qos_profile=0)

    def receiveQueue(self, msg: PlayAudio):
        self.mgr.new_queue = True
        self.mgr.queue = [
            {
                'text': message.text,
                'b64audio': message.b64audio,
                'is_pause': message.is_pause,
                'is_move': message.is_move_command,
                'emotion': message.emotion
            } for message in msg.messages]
        self.mgr.release_mic = msg.release_mic

    def changeSpeakingState(self, msg):
        self.get_logger().info(f'Speaking: {msg.data}')
        self.mgr.speaking = msg.data

    # TODO: control emotions from ROS individually?!
    # def callback(self, msg):
    #     self.get_logger().info(f'New emotion: {msg.data}')
    #     self.mgr.emotion = msg.data


def init_node(mgr, args=None):
    rclpy.init(args=args)
    node = ROSCom(mgr)
    return node
