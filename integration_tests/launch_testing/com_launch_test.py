#!/usr/bin/env python
# based on https://github.com/ros2/launch_ros/blob/master/
# launch_testing_ros/test/examples/check_msgs_launch_test.py

# ROS 2 basics
from rclpy.task import Future
import rclpy

# ROS 2 launchfile
import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions

# unit testing and pytest
import pytest
from threading import Event
from threading import Thread
import unittest

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


class TestFixture(unittest.TestCase):
    def spin(self):
        try:
            while rclpy.ok() and not self.spinning.is_set():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            return

    def setUp(self):
        self.clients = []
        mgr, app = init_websocket()
        self.app = app
        self.node = mgr.ros_node
        # we received three and five from the server
        self.received_three = Event()
        self.received_five = Event()

        self.spinning = Event()
        # Add a spin thread
        self.ros_spin_thread = Thread(target=self.spin)
        self.ros_spin_thread.start()

    def wait_for_service(self, service_clz, service_name):
        """Helper function to wait for a service to become available."""
        client = self.node.create_client(service_clz, service_name)
        service_available = False
        for _try in range(10):
            if client.wait_for_service(timeout_sec=.5):
                service_available = True
                self.node.get_logger().info(
                    f'service {service_name} is available 👍!')
                break
            self.node.get_logger().info(
                f'service not available {service_name}, waiting again...')
        if not service_available:
            raise RuntimeError(f'Service "{service_name}" not available ☠!')
        return client

    def tearDown(self):
        self.spinning.set()
        self.ros_spin_thread.join()
        for cli in self.clients:
            self.node.destroy_client(cli)
        self.node.destroy_node()
        rclpy.shutdown()

    def test_websocket_called(self, proc_output: ActiveIoHandler):
        """the only test: make sure that the websocket has been executed."""
        # check if we received 3 or 5 from the websocket
        client = TestClient(self.app)
        with client.websocket_connect("/ws") as websocket:
            data = websocket.receive_json()
            assert data == {"msg": "Hello WebSocket"}
        # (and also that we are speaking?)
        assert True
