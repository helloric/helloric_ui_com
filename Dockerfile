ARG ROS_DISTRO
FROM d-reg.hb.dfki.de/robot-config/ros-pip-pytest:${ROS_DISTRO}-0.0.1

EXPOSE 7000

ENV APP=/app/helloric_ui_com

COPY ./requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt

COPY README.md ${APP}/README.md
COPY run.bash ${APP}/run.bash
COPY ./helloric_ui_com ${APP}/helloric_ui_com
COPY ./ric-messages ${APP}/ric-messages
WORKDIR ${APP}
RUN chmod +x ./run.bash

ENTRYPOINT ["./run.bash"]