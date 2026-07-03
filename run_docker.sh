#!/bin/bash

touch -a /home/$USER/.ros_docker_bash_history
mkdir -p ws/build ws/install ws/log ws/data

compose_files=(-f docker-compose.yml)
if [[ $(uname -i) = "aarch64" ]]; then
	compose_files+=(-f docker-compose.aarch64.yml)
elif [[ $(uname -i) = "x86_64" ]]; then
	compose_files+=(-f docker-compose.x86_64.yml)
fi

# check, if a container with dfki_quad label is already there
container_id=$(docker ps -aq --filter "label=dfki_quad")

if [[ $container_id = "" ]]; then
	# start new container if there is none
	xhost +local:
	docker compose "${compose_files[@]}" up -d
	docker attach dfki_quad
else
	# restart existing container
	docker start -i $(docker ps -n1 -aq --filter "label=dfki_quad")
fi

exit 0
