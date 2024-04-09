# Specify ROS version for the image
FROM ros:foxy

# Set bash as default shell
SHELL ["/bin/bash", "-c"]

# Set users for the container
RUN useradd -m -s /bin/bash user
USER root

######################################################################################################
#                                            BUILD                                                   #
######################################################################################################

# Add the ROS 2 underlay source to the .bashrc startup file
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Create a workspace and download the micro-ROS tools
RUN mkdir /root/microros_ws
WORKDIR /root/microros_ws
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    apt update && rosdep update && \
    rosdep install --from-paths src --ignore-src -y

# Install pip
RUN apt-get update  && \
    apt-get install -y python3-pip

# Build micro-ROS tools and source them
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build && \
    source install/local_setup.sh

# Add the ROS 2 overlay source to the .bashrc startup file
RUN echo "source /root/microros_ws/install/setup.bash" >> ~/.bashrc

######################################################################################################

# Clone the PX4 source code
RUN git clone https://github.com/PX4/px4_ros_com.git src/px4_ros_com && \
    git clone https://github.com/PX4/px4_msgs.git src/px4_msgs

# Move and build it
RUN source /root/microros_ws/install/setup.bash && \
    colcon build

######################################################################################################

# Download and build the micro-ROS agent
RUN source /root/microros_ws/install/setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh && \
    source /root/microros_ws/install/setup.bash
    
######################################################################################################

# Install wget for PX4 setup
RUN apt-get update  && \
    apt-get install -y wget

# Clone the PX4 repository and its dependencies
WORKDIR /root
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Setup of PX4 on Ubuntu for simulation purposes
RUN bash /root/PX4-Autopilot/Tools/setup/ubuntu.sh

# Build PX4
RUN cd /root/PX4-Autopilot && \
    DONT_RUN=1 make px4_sitl_default gazebo-classic

######################################################################################################

# Download and install QGroundControl
RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage && \
    chmod +x QGroundControl.AppImage

######################################################################################################

# Install utilities
RUN apt-get update  && \
    #apt-get install -y sudo && \
    apt-get install -y vim && \
    apt-get install -y tmux && \
    #apt-get install -y psmisc && \
    apt-get install -y tmuxinator

######################################################################################################
#                                            STARTUP                                                 #
######################################################################################################

# Set starting directory
WORKDIR /root

# Copy and set up the entrypoint script and tmux configs
COPY ./entrypoint.sh .
RUN chmod +x entrypoint.sh

COPY ./tmux ./tmux
RUN chmod +x ./tmux/*.sh

# Set entrypoint as startup script
ENTRYPOINT ["./entrypoint.sh"]