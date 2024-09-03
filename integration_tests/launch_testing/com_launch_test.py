#!/usr/bin/env python
# based on https://github.com/ros2/launch_ros/blob/master/
# launch_testing_ros/test/examples/check_msgs_launch_test.py
import asyncio

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
from threading import Event
from threading import Thread
import aiounittest

# launch_testing
from launch_testing.io_handler import ActiveIoHandler
import launch_testing.markers

# see https://fastapi.tiangolo.com/advanced/testing-websockets/
from fastapi.testclient import TestClient

from helloric_ui_com.main import init_websocket


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


class TestFixture(aiounittest.AsyncTestCase):
    def setUp(self):
        self.spinning = Event()
        self.clients = []
        mgr, app = init_websocket()
        self.app = app
        self.mgr = mgr
        self.node = mgr.ros_node

    async def test_websocket_called(self):
        """the only test: make sure that the websocket has been executed."""
        event_loop = asyncio.get_event_loop()
        asyncio.ensure_future(self.mgr.main_loop(), loop=event_loop)
        client = TestClient(self.app)
        received_emo = received_speak = False
        with client.websocket_connect("/ws") as websocket:
            while not received_emo and not received_speak:
                print('wait for websocket to receive data...')
                data = websocket.receive_json()
                print(f'received: {data}')
                # check if we received 3 or 5 from the websocket
                if 'emotion' in data:
                    if data['emotion'] != 0:
                        assert int(data['emotion']) in [3, 5]
                        received_emo = True
                if 'speaking' in data:
                    received_speak = True
        print(received_emo, received_speak)
        self.node.destroy_node()
        rclpy.shutdown()
        return True