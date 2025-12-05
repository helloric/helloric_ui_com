ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}

RUN apt-get update -qq \
    && apt-get install -y \
    python3-pip \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-diagnostic-msgs \
    # ament-cmake \
    && rm -rf /var/lib/apt/lists/*

# PEP 668: https://docs.ros.org/en/independent/api/rosdep/html/pip_and_pep_668.html
ENV PIP_BREAK_SYSTEM_PACKAGES=1
# install flake and pytest-cov for testing
RUN pip3 install --upgrade flake8 pytest-cov --user

EXPOSE 7000

ENV APP=/app/helloric_ui_com

COPY ./requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt

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
