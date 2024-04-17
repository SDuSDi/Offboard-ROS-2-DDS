#! /bin/bash
# set -e
# echo "This script is trying to run the command 'xhost +xlocal:root'. This command is needed to be able to see Gazebo running from inside th docker. Allow? [Y,n]" | read input_xhost
# if [$input_xhost != "N" -o $input_xhost != "n"]; then xhost +xlocal:root; fi
docker run -it --rm -v /home/bmartinez/workspace/micro_ros2/braismtnez-ros2sim/workspace:/root/workspace --net=host --env DISPLAY=$DISPLAY --privileged braismtnez/ros2dds
# Build command -> docker build -t braismtnez/ros2dds ./workspace/micro_ros2/braismtnez-ros2sim/.