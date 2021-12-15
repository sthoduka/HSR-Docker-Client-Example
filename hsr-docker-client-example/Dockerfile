# syntax=docker/dockerfile:1

# Starting with ROS Melodic
FROM ros:melodic

# Update packages
RUN apt-get update

# Install essentials
RUN apt-get install -y python-catkin-tools
RUN apt-get install -y build-essential
RUN apt-get install -y git
RUN apt-get install -y wget
RUN apt-get install -y python-pip

# Install ros-melodic-* required packages
# e.g.  ros-melodic-image-transport, ros-melodic-cv-bridge
RUN apt-get install -y ros-melodic-controller-manager-msgs 

# Install pip dependencies
RUN pip install pyyaml

# Create the catkin workspace directories
RUN mkdir -p /home/root/catkin_ws/src

# Download dependencies from GitHub
WORKDIR /home/root/Downloads
RUN git clone https://github.com/care-group/HSR-Docker-Client-Example.git

# Move the ROS packages into place
RUN mv HSR-Docker-Client-Example/packages/hsr_head_positioner /home/root/catkin_ws/src
RUN mv HSR-Docker-Client-Example/packages/pointcloud_example_ros_ws /home/root/catkin_ws/src

# Initialise the catkin workspace and catkin_make
RUN . /opt/ros/melodic/setup.sh && catkin_init_workspace /home/root/catkin_ws/src
RUN . /opt/ros/melodic/setup.sh && cd /home/root/catkin_ws && catkin_make

# Copy in the entrypoint script
COPY start.sh /
RUN chmod +x /start.sh
ENTRYPOINT [ "/start.sh" ]
CMD ["start"]
