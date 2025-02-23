#!/bin/sh
#

IMAGE_ESP8266SDK="nozomidop/esp8266-builder:v3.4" # esptool.py included
#IMAGE_ESP8266SDK="xingrz/esp8266-rtos-builder"
IMAGE_FISHBOTTOOL="nozomidop/esp8266-builder:v3.4" # 
#IMAGE_FISHBOTTOOL="fishros2/fishbot-tool"


if [ "$1" == "bash" ]; then
    docker run -it --rm --privileged -v=/dev:/dev  -v `pwd`:`pwd` -w `pwd` ${IMAGE_ESP8266SDK} bash 
fi

if [ "$1" == "make" ]; then
    docker run -it --rm --privileged -v=/dev:/dev  -v `pwd`:`pwd` -w `pwd` ${IMAGE_ESP8266SDK} make 
fi

if [ "$1" == "merge" ]; then
    docker run -it --rm --privileged -v=/dev:/dev  -v `pwd`:`pwd` -w `pwd` ${IMAGE_FISHBOTTOOL} esptool.py \
    --chip ESP8266 merge_bin -o fishbot_laser_control_v1.0.0.`date +%y%m%d`.bin \
    --flash_mode dio --flash_size 4MB \
    0x00 build/bootloader/bootloader.bin 0x8000 build/partitions_singleapp.bin 0x10000 build/uart2udp.bin
fi

if [ "$1" == "flash" ]; then
    docker run -it --rm --privileged -v=/dev:/dev  -v `pwd`:`pwd` -w `pwd` ${IMAGE_ESP8266SDK} make 
fi

echo "fishbot-laser-control($1)...DONE"
echo ""
