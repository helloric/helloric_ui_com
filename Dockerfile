ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}

ENV COLCON_WS=/root/colcon_ws
ENV COLCON_WS_SRC=/root/colcon_ws/src
ENV PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR ${COLCON_WS}

# TODO: copy messages from helloric-
# TODO: copy entrypoint to source workspace

# build everything external first so docker can cache it
RUN cd ${COLCON_WS}\
    && . /opt/ros/${ROS_DISTRO}/setup.sh\
    && colcon build

# Install additional apt packages
RUN apt-get update -qq \
    && apt-get install -y \
        python3-pip \
        python3-argcomplete \
        python3-pip \
        python3-netifaces \
        python3-yaml \
        python3-distro \
    && rm -rf /var/lib/apt/lists/*


EXPOSE 7000

COPY ./requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt

COPY ./helloric_ui_com /app/helloric_ui_com

WORKDIR /app/helloric_ui_com

CMD ["python3", "cli.py"]