#!/bin/bash
docker compose build
docker compose run helloric_ui_com bash -c "source ~/colcon_ws/install/setup.bash && launch_test /app/helloric_ui_com/integration_tests/launch_testing/com_launch_test.py"
