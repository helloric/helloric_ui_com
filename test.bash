#!/bin/bash
docker compose build
docker compose run helloric_ui_com bash -c "source /opt/ros/humble/setup.bash && launch_test /integration_tests/launch_testing/com_launch_test.py"
