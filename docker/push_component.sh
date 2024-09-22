#!/bin/sh

export DOCKER_CLI_EXPERIMENTAL=enabled


docker buildx create --name nimbus-builder 
docker buildx use nimbus-builder
docker run --privileged --rm tonistiigi/binfmt --install all
docker buildx inspect --bootstrap



sudo docker buildx build --platform linux/arm64,linux/amd64 -t cognimbus/data-publsiher-ros2:humble --push .

