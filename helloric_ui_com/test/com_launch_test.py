#!/usr/bin/env python
# based on https://github.com/ros2/launch_ros/blob/master/
# launch_testing_ros/test/examples/check_msgs_launch_test.py

# Python basics to launch helloric_ui_com
import asyncio
from threading import Event
from threading import Thread

# ROS 2 basics
import rclpy
from std_msgs.msg import Bool, Int8
from ric_messages.msg import PlayAudio, LLMAudioResponse

# ROS 2 launchfile
import launch
import launch.actions
import launch_testing.actions
import launch_testing.markers

# unit testing and pytest
import pytest
import unittest

# see https://fastapi.tiangolo.com/advanced/testing-websockets/
from fastapi.testclient import TestClient

# custom init_websocket to get app that we use for mocking
from helloric_ui_com.main import init_websocket


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        launch_testing.actions.ReadyToTest()
    ])

@pytest.mark.skip
def assert_queue_equals(expected: PlayAudio, actual):
    assert 'release_mic' in actual
    assert expected.release_mic == actual['release_mic']

    assert 'messages' in actual
    msgs = actual['messages']
    assert len(expected.messages) == len(msgs)

    for i, elem in enumerate(msgs):
        assert_message_equals(expected.messages[i], elem)

@pytest.mark.skip
def assert_message_equals(expected: LLMAudioResponse, actual):
    assert 'b64audio' in actual
    assert 'emotion' in actual
    assert 'text' in actual
    assert 'is_pause' in actual
    assert 'is_move' in actual

    assert expected.b64audio == actual['b64audio']
    assert expected.emotion == actual['emotion']
    assert expected.text == actual['text']
    assert expected.is_pause == actual['is_pause']
    assert expected.is_move_command == actual['is_move']


class TestFixture(unittest.TestCase):
    async def _spin(self):
        while rclpy.ok() and not self.spinning.is_set():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            await self.mgr.update_data()

    def spin_thread_wrapper(self):
        """a thread to run a coroutine.
        
        We need to call asyncio-broadcast that requires async."""
        self.ros_spin_loop.run_until_complete(self._spin())


    def setup_ros(self):
        self.spinning = Event()
        self.clients = []
        mgr, app = init_websocket()
        self.app = app
        self.mgr = mgr
        self.node = mgr.ros_node

        self.publisher_queue = self.node.create_publisher(
            PlayAudio, 'websocket/play_audio', 10)
        self.publisher_emo = self.node.create_publisher(
            Int8, 'llm/speech/emotion', 10)
        self.publisher_speak = self.node.create_publisher(
            Bool, 'llm/speech/speaking', 10)

    def setUp(self):
        self.setup_ros()
        self.spinning = Event()
        self.ros_spin_loop = asyncio.new_event_loop()
        self.ros_spin_thread = Thread(target=self.spin_thread_wrapper)
        self.ros_spin_thread.start()

    def tearDown(self):
        self.spinning.set()
        self.ros_spin_thread.join()
        self.node.destroy_node()
        rclpy.shutdown()

    def test_websocket_called(self):
        """the only test: make sure that the websocket has been executed."""
        client = TestClient(self.app)
        with client.websocket_connect("/ws") as websocket:
            # we did not send anything yet, the websocket message should be 0 first
            ws_msg = websocket.receive_json()
            assert 'emotion' in ws_msg
            assert 'speaking' in ws_msg
            assert ws_msg['emotion'] == 0
            assert not ws_msg['speaking']

            # The stuff we send should eventually give out the same values
            msg = PlayAudio()
            resp1 = LLMAudioResponse()
            resp1.emotion = 8
            msg.messages = [resp1]

            self.publisher_queue.publish(msg)

            ws_msg = websocket.receive_json()
            
            assert_queue_equals(msg, ws_msg)

            msg = PlayAudio()
            resp1 = LLMAudioResponse()
            resp1.emotion = 2
            resp1.text = 'I am feeling rather well.'
            resp1.b64audio = 'This is a totally legit audio'
            resp1.is_move_command = False
            resp1.is_pause = False
            resp2 = LLMAudioResponse()
            resp2.text = '_pause'
            resp2.is_pause = True
            resp3 = LLMAudioResponse()
            resp3.b64audio = 'Also a very legit audio'
            resp3.text = 'How about you?'
            msg.release_mic = True
            msg.messages = [resp1, resp2, resp3]

            self.publisher_queue.publish(msg)

            ws_msg = websocket.receive_json()

            assert_queue_equals(msg, ws_msg)