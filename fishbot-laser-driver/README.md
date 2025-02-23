### retrieve fishbot_laser from docker container fishros2/fishbot_laser
[source](https://hub.docker.com/r/fishros2/fishbot_laser)
1. docker pull fishros2/fishbot_laser:latest
2. copy container/workspace out

#### How to 
1. docker launch fishbot-laser-driver
"""
docker run -it --rm -v /home/kin95man/fishros/fishbot-laser-driver:/fishbot-laser-driver fishros2/fishbot_laser ls
"""

2. docker exec into shell
"""
# to find the container name
docker ps | grep fish
# replace relaxed_merkle with the correct name
docker exec -it relaxed_merkle bash
"""