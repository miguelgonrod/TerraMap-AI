#!/bin/bash

docker build -f build/turtle.Dockerfile -t turtleimage .
docker run --rm -it \
	--env="DISPLAY" \
    	--env="QT_X11_NO_MITSHM=1" \
    	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--network host \
	-v mapsVolume:/maps \
	turtleimage:latest
