#!/usr/bin/env python
# based on https://github.com/ros2/launch_ros/blob/master/
# launch_testing_ros/test/examples/check_msgs_launch_test.py

# Python basics to launch helloric_ui_com
import subprocess

from threading import Event
from threading import Thread

# Websocket-client library to test connection
import json
import websocket

# ROS 2 basics
import rclpy
from std_msgs.msg import Bool, Int8

# ROS 2 launchfile
import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions

# unit testing and pytest
import pytest
import unittest

# launch_testing
import launch_testing.markers


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        launch_testing.actions.ReadyToTest()
    ])



class TestFixture(unittest.TestCase):
    def spin(self):
        try:
            while rclpy.ok() and not self.spinning.is_set():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            pass

    def start_helloric_ui_com(self):
        return

    def setup_ros(self):
        rclpy.init()
        self.node = rclpy.create_node('test_node')

        self.publisher_emo = self.node.create_publisher(
            Int8, 'llm/speech/emotion', 10)
        self.publisher_speak = self.node.create_publisher(
            Bool, 'llm/speech/speaking', 10)

        self.spinning = Event()
        # Add a spin thread
        self.ros_spin_thread = Thread(target=self.spin)
        self.ros_spin_thread.start()

    def setUp(self):
        self.setup_ros()

        # start server
        self.websocket_response = Event()
        self.helloric_ui_com = subprocess.Popen(
            ['helloric_ui_com', '--host', '0.0.0.0', '--port', '8260'])

    def tearDown(self):
        if self.helloric_ui_com:
            self.helloric_ui_com.kill()
        self.spinning.set()
        self.ros_spin_thread.join()
        self.node.destroy_node()
        rclpy.shutdown()


    def receive_websocket(self):
        return json.loads(self.ws.recv())

    def test_websocket_called(self):
        """the only test: make sure that the websocket has been executed."""
        
        # TODO: this should be a ROS node that send a message and receives
        # the emotion from the given websocket
        
        uri = "ws://localhost:8260/ws"
        print(f'connecting to {uri}')
        self.ws = websocket.WebSocket()
        connected = False
        while not connected:
            try:
                self.ws.connect(uri)
                connected = True
            except ConnectionRefusedError:
                pass
        # we did not send anything yet, the websocket message should be 0 first
        ws_msg = self.receive_websocket()
        assert 'emotion' in ws_msg
        assert ws_msg['emotion'] == 0

        # if we send 5 we should receive 5
        msg = Int8()
        msg.data = 5
        self.publisher_emo.publish(msg)

        ws_msg = self.receive_websocket()
        assert 'emotion' in ws_msg
        assert ws_msg['emotion'] == 5



