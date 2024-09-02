ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}

WORKDIR ${COLCON_WS}

EXPOSE 7000

COPY ./requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt

COPY ./helloric_ui_com /app/helloric_ui_com

WORKDIR /app/helloric_ui_com

CMD ["python3", "cli.py"]