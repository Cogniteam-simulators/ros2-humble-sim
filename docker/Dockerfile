from ros:humble

WORKDIR ros2_humble_sim_ws/src

RUN git clone https://github.com/Cogniteam-simulators/ros2-humble-sim.git


WORKDIR ros2_humble_sim_ws/

RUN apt-get update && \
    apt install ros-humble-nav-msgs -y && \
    apt install ros-humble-geometry-msgs -y && \
    apt install ros-humble-sensor-msgs -y && \
    apt install ros-humble-rviz2 -y && \
    apt install ros-humble-vision-opencv -y && \
    apt install ros-humble-cv-bridge -y  && \
    apt install ros-humble-navigation2 -y  && \
    apt install ros-humble-nav2-bringup -y


RUN apt install python3-pip -y
RUN apt install ros-humble-tf-transformations -y
RUN  pip3 install pyyaml


RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


CMD ros2 launch ros2_robot_sim bringup_launch.py


