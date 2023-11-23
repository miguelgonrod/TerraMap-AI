#!/bin/bash

docker build -f build/server.Dockerfile -t serverimage .
docker run \
	--detach \
	--name=terraSQL \
	--env="MYSQL_ROOT_PASSWORD=fW25>a£8:URB" \
	--publish 6603:3306 \
	--volume=/storage/docker/mysql-data:/var/lib/mysql \
	mysql

