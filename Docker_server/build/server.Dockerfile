FROM mysql/mysql-server:latest

ENV MYSQL_ROOT_PASSWORD=fW25>a£8:URB

EXPOSE 3306

ENV MYSQL_DATABASE=terraSQL

VOLUME /var/lib/mysql

VOLUME /storage/docker/mysql-data:/var/lib/mysql