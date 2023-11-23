FROM osrf/ros:noetic-desktop-full

RUN apt-get update && \
    apt-get install -y python3-pip && \
    apt-get install -y nano && \
    apt-get install -y tmux && \
    apt-get install -y net-tools && \
    apt-get install -y ros-noetic-turtlebot3* && \
    apt-get install -y ros-noetic-slam*

RUN useradd -m -s /bin/bash -N -u 1000 turtle && \
    echo "turtle ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers && \
    chmod 0440 /etc/sudoers && \
    chmod g+w /etc/passwd 

USER turtle

WORKDIR home/turtle/app

COPY --chown=turtle app/ .

ENV TURTLEBOT3_MODEL burger

RUN chmod 774 ~/app/turtle_ws/

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash;"

