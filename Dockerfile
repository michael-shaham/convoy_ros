FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

# dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       nano \
                       vim \
                       python3-pip \
                       libeigen3-dev \
                       tmux
RUN apt-get -y dist-upgrade

RUN pip3 install transforms3d scikit-learn cvxpy

# f1tenth gym
RUN git clone https://github.com/f1tenth/f1tenth_gym.git
RUN cd f1tenth_gym && \
    pip3 install -e .

# convoy (includes gym bridge)
RUN mkdir -p /root/cvy_ws/src/convoy_ros/
COPY . /root/cvy_ws/src/convoy_ros/
RUN source /opt/ros/foxy/setup.bash && \
    cd /root/cvy_ws/ && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro foxy -y && \
    colcon build --symlink-install

WORKDIR '/root/cvy_ws'
ENTRYPOINT ["/bin/bash"]
