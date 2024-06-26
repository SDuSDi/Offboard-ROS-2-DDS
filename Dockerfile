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

WORKDIR /root

# Instalation of prerequisites for MQTT
RUN apt-get update && \
    apt-get install -y build-essential gcc make cmake cmake-gui cmake-curses-gui && \
    apt-get install -y libssl-dev && \
    apt-get install -y doxygen graphviz && \
    apt install -y git-all

# Install Paho MQTT C and build it
RUN git clone https://github.com/eclipse/paho.mqtt.c.git && \
    cd paho.mqtt.c && \
    git checkout v1.3.13 && \
    cmake -Bbuild -H. -DPAHO_ENABLE_TESTING=OFF -DPAHO_BUILD_STATIC=ON -DPAHO_WITH_SSL=ON -DPAHO_HIGH_PERFORMANCE=ON && \
    cmake --build build/ --target install && \
    ldconfig

# Install Paho MQTT C++ and build it
RUN git clone https://github.com/eclipse/paho.mqtt.cpp && \
    cd paho.mqtt.cpp && \
    cmake -Bbuild -H. -DPAHO_WITH_MQTT_C=OFF -DPAHO_BUILD_STATIC=ON -DPAHO_BUILD_DOCUMENTATION=ON -DPAHO_BUILD_SAMPLES=ON && \
    cmake --build build/ --target install && \
    ldconfig

######################################################################################################

# Install nlohmann JSON library
RUN apt-get update && apt-get install nlohmann-json3-dev

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
    apt-get install -y python3-pip && \
    pip install colcon-clean

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

# Set work directory for QGroundControl
WORKDIR /home/user
RUN apt-get install -y fuse libpulse-mainloop-glib0 libfuse2

# Fix problems with QGroundControl
RUN usermod -a -G dialout user && \
    apt-get remove modemmanager

# Download and install QGroundControl
RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage && \
    chmod +x QGroundControl.AppImage

######################################################################################################

# Install utilities
RUN apt-get update  && \
    apt-get install -y x11-apps && \
    apt-get install -y vim && \
    apt-get install -y tmux && \
    apt-get install -y kmod && \
    apt-get install -y tmuxinator

######################################################################################################
#                                            STARTUP                                                 #
######################################################################################################

# Set starting directory
WORKDIR /root

# Create user workspace
RUN mkdir workspace

# Copy and set up the entrypoint script and tmux configs
COPY ./entrypoint.sh .
RUN chmod +x entrypoint.sh

COPY ./tmux ./tmux
RUN chmod +x ./tmux/*.sh

# Set entrypoint as startup script
ENTRYPOINT ["./entrypoint.sh"]