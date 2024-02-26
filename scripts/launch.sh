#!/bin/bash
SERVER_IMAGE=${SERVER_IMAGE:-rmus2022/server:result_pub_fix}
CLIENT_IMAGE=${CLIENT_IMAGE:-client-custom:latest} # Just for test, if not effect, please change it to your client docker image. 
# SERVER_IMAGE=${SERVER_IMAGE:-rmus2022/server:v1.0.0}
# CLIENT_IMAGE=${CLIENT_IMAGE:-client-custom:latest}
CLI_EXE=$@

xhost +

# docker pull $SERVER_IMAGE

docker network create net-sim

docker run -dit --rm --name ros-master --network net-sim ros:noetic-ros-core-focal roscore

docker run -dit --rm --name sim-server --network net-sim \
	--gpus all \
	-e ROS_MASTER_URI=http://ros-master:11311 \
	-e DISPLAY=$DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	-e NO_AT_BRIDGE=1 \
	-e LIBGL_ALWAYS_SOFTWARE=1 \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	$SERVER_IMAGE 

sleep 2
echo "server is finished"

docker run -it --rm --name client --network net-sim \
	--gpus all \
	--cpus=7.0 -m 8192M \
	-e ROS_MASTER_URI=http://ros-master:11311 \
	-e DISPLAY=$DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	-e NO_AT_BRIDGE=1 \
	-e LIBGL_ALWAYS_SOFTWARE=1 \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v ./src/rmus_solution:/opt/ep_ws/src/rmus_solution \
	-v ./src/navigation:/opt/ep_ws/src/navigation \
	--privileged=true -u=root \
	$CLIENT_IMAGE $CLI_EXE


## navigation debug:
#	-v ./src/navigation:/opt/ep_ws/src/navigation \
## rviz:
#	--gpus all \
