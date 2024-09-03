ARG ROS_DISTRO
FROM d-reg.hb.dfki.de/robot-config/ros-pip-pytest:${ROS_DISTRO}-0.0.1

EXPOSE 7000

ENV COLCON_WS=/root/colcon_ws
ENV COLCON_WS_SRC=/root/colcon_ws/src
ENV APP=/app/helloric_ui_com

COPY ./requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt

COPY setup.cfg ${APP}/setup.cfg
COPY setup.py ${APP}/setup.py
COPY README.md ${APP}/README.md
COPY ./helloric_ui_com ${APP}/helloric_ui_com
WORKDIR ${APP}
RUN pip3 install .

COPY ./integration_tests /integration_tests

RUN mkdir -p ${COLCON_WS_SRC} && \
    ln -s /integration_tests/helloric_ui_com_test ${COLCON_WS_SRC}/helloric_ui_com_test

WORKDIR ${COLCON_WS}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

WORKDIR ${APP}

CMD ["helloric_ui_com"]