# Use the official ROS 2 Humble base image
FROM ros:humble

# Set the working directory inside the container
WORKDIR /ros2_humble_sim_ws/

# Clone the necessary repository into the src folder
RUN mkdir -p src && \
    cd src && \
    git clone https://github.com/Cogniteam-simulators/ros2-humble-sim.git && \
    cd ..

# Install required ROS 2 packages
RUN apt-get update && \
    apt-get install -y \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-rviz2 \
    ros-humble-vision-opencv \
    ros-humble-cv-bridge \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-tf-transformations \
    python3-pip


RUN  pip3 install pyyaml

# Source the ROS environment and build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Copy the entrypoint script into the container
COPY entrypoint.sh /entrypoint.sh

# Make the entrypoint script executable
RUN chmod +x /entrypoint.sh

# Set the entrypoint script to launch your ROS 2 application
ENTRYPOINT ["/entrypoint.sh"]

# Default command to run if none is provided
CMD ["ros2", "launch", "ros2_robot_sim", "bringup_launch.py"]
