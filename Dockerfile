ARG ROS_DISTRO
FROM d-reg.hb.dfki.de/robot-config/ros-pip-pytest:${ROS_DISTRO}-0.0.1

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