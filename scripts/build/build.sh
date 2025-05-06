#!/bin/bash
docker build --rm  $@ -t mppi_ros2:latest -f "$(dirname "$0")/../../docker/mppi_docker.Dockerfile" "$(dirname "$0")/../.."