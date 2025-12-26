#!/bin/bash
PROJECT_ROOT="$PWD"
# absolute_path=/home/prajwal/projects/mppi_docker_for_ROS2_Simulation/
run_docker() {
    # -it is for interactive, tty
    # --privileged for accessing /dev contents
    # --net=host to share the same network as host machine. TL;DR same IP.
    xhost +local:root # giving display privilages
    docker run -it --privileged --net=host \
    --ipc=host \
    --name mppi_docker \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ${PROJECT_ROOT}/scripts/deploy/app.sh:/root/app.sh \
    $@
}



stop_docker() {
    docker stop mppi_docker && docker rm mppi_docker
}
