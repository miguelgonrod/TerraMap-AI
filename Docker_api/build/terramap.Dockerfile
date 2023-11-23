FROM osrf/ros:noetic-desktop-full

RUN apt-get update && \
    apt-get install -y python3-pip && \
    apt-get install -y nano && \
    apt-get install -y tmux && \
    apt-get install -y net-tools && \
    apt-get install -y python3-testresources && \
    pip install google-auth google-auth-oauthlib google-auth-httplib2 google-api-python-client 

RUN useradd -m -s /bin/bash -N -u 1000 terra && \
    echo "terra ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers && \
    chmod 0440 /etc/sudoers && \
    chmod g+w /etc/passwd 

USER terra

WORKDIR home/terra/app

COPY --chown=terra app/ .

RUN chmod 774 ~/app/terra_ws/

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash;"

