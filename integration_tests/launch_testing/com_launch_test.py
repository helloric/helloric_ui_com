#!/usr/bin/env python
# based on https://github.com/ros2/launch_ros/blob/master/
# launch_testing_ros/test/examples/check_msgs_launch_test.py

# Python basics to launch helloric_ui_com
import subprocess

# Websocket-client library to test connection
import json
import websocket

# ROS 2 basics
import rclpy

# ROS 2 launchfile
import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions

# unit testing and pytest
import pytest
from threading import Event
import unittest

# launch_testing
import launch_testing.markers


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='helloric_ui_com_test',
            executable='emotion_publisher',
            name='emotion_publisher',
            parameters=[]
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestFixture(unittest.TestCase):
    def start_helloric_ui_com(self):
        return

    def setUp(self):
        # start server
        self.websocket_response = Event()
        self.helloric_ui_com = subprocess.Popen(
            ['helloric_ui_com', '--host', 'localhost'])

    def tearDown(self):
        if self.helloric_ui_com:
            self.helloric_ui_com.kill()

    def on_message(self, msg):
        print('msg:', msg)
        self.websocket_response.set()
        self.helloric_ui_com.kill()
        self.helloric_ui_com = None

    def test_websocket_called(self):
        """the only test: make sure that the websocket has been executed."""
        
        # TODO: this should be a ROS node that send a message and receives
        # the emotion from the given websocket
        
        uri = "ws://localhost:7000/ws"
        print(f'connecting to {uri}')
        self.ws = websocket.WebSocket()
        connected = False
        while not connected:
            try:
                self.ws.connect(uri)
                connected = True
            except ConnectionRefusedError:
                pass
        msg_received = False
        tries = 0
        while not msg_received:
            if tries > 15:
                break
            msg = json.loads(self.ws.recv())
            print(msg)
            if 'emotion' in msg and msg['emotion'] in [3, 5]:
                msg_received = True
            tries += 1
        assert msg_received, 'No Webservice response'

