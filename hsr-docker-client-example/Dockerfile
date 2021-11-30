# syntax=docker/dockerfile:1

# Starting with ROS kinetic
FROM ros:kinetic

# Update packages
RUN apt-get update

# Install essentials
RUN apt-get install -y python-catkin-tools
RUN apt-get install -y build-essential
RUN apt-get install -y git
RUN apt-get install -y wget
RUN apt-get install -y python-pip
# RUN apt-get install -y [ros-kinetic-*] # install required ros packages, e.g.  ros-kinetic-image-transport, ros-kinetic-cv-bridge

# Create the catkin workspace directories
RUN mkdir -p /home/root/catkin_ws/src

# Download dependencies from GitHub
WORKDIR /home/root/Downloads
RUN git clone https://github.com/care-group/HSR-Docker-Client-Example.git

# Move the ROS packages into place
RUN mv HSR-Docker-Client-Example/packages/hsr_head_positioner /home/root/catkin_ws/src

# Initialise the catkin workspace and catkin_make
RUN . /opt/ros/kinetic/setup.sh && catkin_init_workspace /home/root/catkin_ws/src
RUN . /opt/ros/kinetic/setup.sh && cd /home/root/catkin_ws && catkin_make

# Copy in the entrypoint script
COPY start.sh /
RUN chmod +x /start.sh
ENTRYPOINT [ "/start.sh" ]
CMD ["start"]
