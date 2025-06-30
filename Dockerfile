ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}

RUN apt-get update -qq \
    && apt-get install -y \
    python3-pip \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-diagnostic-msgs \
    ros-${ROS_DISTRO}-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

# install flake and pytest-cov for testing
# We are inside a docker container so its okay to "break-system-packages".
RUN pip3 install --upgrade flake8 pytest-cov --break-system-packages

EXPOSE 7000

ENV APP=/app/helloric_ui_com

COPY ./requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt --break-system-packages

WORKDIR ${APP}
COPY ./ric-messages ${APP}/ric-messages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build

COPY entrypoint.bash /entrypoint.bash
RUN chmod +x /entrypoint.bash

COPY README.md ${APP}/README.md
COPY ./helloric_ui_com ${APP}/helloric_ui_com
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build

ENTRYPOINT ["/entrypoint.bash"]
CMD ["ros2", "run", "helloric_ui_com", "helloric_ui_com"]
