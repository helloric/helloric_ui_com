ARG ROS_DISTRO
FROM d-reg.hb.dfki.de/robot-config/ros-pip-pytest:${ROS_DISTRO}-0.0.1

WORKDIR ${COLCON_WS}

EXPOSE 7000

COPY ./requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt

COPY ./helloric_ui_com /app/helloric_ui_com
COPY ./integration_tests /integration_tests

RUN mkdir -p ~/colcon_ws/src && \
    ln -s /integration_tests/helloric_ui_com_test ~/colcon_ws/src/helloric_ui_com_test

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && cd ~/colcon_ws && colcon build

WORKDIR /app/helloric_ui_com

# TODO: copy/build helloric_ui_com_test!

#COPY ./run_tests.bash /run_tests.bash
#RUN chmod +x /run_tests.bash

# comment this out if you want to run the tests as default instead!
# CMD [ "bash", "-c", "/run_tests.bash"]

CMD ["python3", "cli.py"]