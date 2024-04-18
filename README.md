# Offboard ROS 2 DDS

<!--Nah, I'd push-->
This project contains the code for offboard control with ROS 2 made during my curricular end-of-studies internship at CATEC using DDS to communicate with PX4's autopilot.

<!--![image](assets/iris_pocho.png)-->
![image](assets/schema.png)

## Dependencies and prerequisites

To use the code hosted on this repo, it is required that you use <a href="https://www.docker.com/" target="_blank" rel="noreferrer">Docker</a>. To install Docker Engine, which is the required part, you can find the instructions <a href="https://docs.docker.com/engine/install/ubuntu/" target="_blank" rel="noreferrer">here</a>.

After you are done installing docker, you'll need to clone the DLStreamer Application repository to control the drone. That repository can be found <a href="https://github.com/SDuSDi/DLStreamer-MQTT-Dockered-Application" target="_blank" rel="noreferrer">here</a>. Follow the instructions and keep it up during the usage of this ROS 2 repository. Make sure to center your face in the middle of the frame and to always have at least one face visible to the camera, to keep the drone steady in its position.

## Usage

<!--TODO: Arreglar la primer parte-->
To use, the first step is to clone the repository. You may do this with the following command.
```
git clone https://github.com/SDuSDi/Offboard-ROS-2-DDS.git
```
The next step should be to build the docker, since all the implementation was tested on it. The command to build the docker is commented inside the <em>docker_run.sh</em> file. This takes around 14 minutes. If you didn't add you user to the docker group, you'll have tu use sudo before each docker command. To fix this, follow the steps in this <a href="https://www.linkedin.com/pulse/how-run-docker-commands-without-sudo-andrey-byhalenko-gawzf" target="_blank" rel="noreferrer">tutorial</a>.
```
docker build -t braismtnez/ros2dds ./Offboard-ROS-2-DDS/.
```
The docker runs internally gazebo, PX4 and a ground control station. To be able to see this running inside the docker, you may need to execute this command previous to running the docker. The command gives access to the grafical interface so that you may see the programs running inside the docker.
```
xhost +xlocal:root
```
To run said docker, the following command should be ran in the same terminal as the build one.
```
docker run -it --rm -v ./Offboard-ROS-2-DDS/workspace:/root/workspace --net=host --env DISPLAY=$DISPLAY --privileged braismtnez/ros2dds
```
<b>Congratulations!</b> If everything is alright, you are now inside the docker.

### Inside the docker

Now that you are inside the docker, you will see that its divided in 4 "mini" terminals. You can move around in those with you cursor. On the bottom of the terminal, you'll see that you start in window nº 3. You can move from one window to another with Ctrl+B+(nº), but for the purpose of this tutorial, it won't be necessary. Select one of the terminals in window nº3 and use the following commands.
```
cd workspace
source install/setup.bash

# Be sure to wait until both Gazebo and QGroundControl are up and running smoothly
ros2 run drone node
```
If everything is alright, you should see the drone takeoff and redirect itself to (0,0) from where it is. If you move your head so that it gets near an edge of the camera, the drone will move accordingly.

## Diving Deeper

- https://bitbucket.org/fadacatec-ondemand/px4_ros2_simulation 
- https://docs.px4.io/main/en/middleware/uxrce_dds.html#foxy
- https://github.com/Jaeyoung-Lim/px4-offboard/blob/2d784532fd323505ac8a6e53bb70145600d367c4/doc/ROS2_PX4_Offboard_Tutorial.md

## Contact

Brais Martínez -> bmartinez.ext@catec.aero