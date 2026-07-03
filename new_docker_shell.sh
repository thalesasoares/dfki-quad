#!/bin/bash

container_id=$(docker ps -q --filter "label=dfki_quad")

if [[ $container_id = "" ]]; then
	echo "No running dfki_quad container found. Start one with ./run_docker.sh first."
	exit 1
fi

docker exec -it "$container_id" /bin/bash

exit 0
