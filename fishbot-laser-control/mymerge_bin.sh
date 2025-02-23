#!/bin/bash
#

# pip install esptool
ESPTOOL=/root/.local/lib/python2.7/site-packages/esptool.py
chmod +x ${ESPTOOL}
${ESPTOOL} \
    --chip ESP8266 merge_bin -o fishbot_laser_control_v1.0.1.`date +%y%m%d`.bin \
    --flash_mode dio --flash_size 4MB \
    0x00 build/bootloader/bootloader.bin 0x8000 build/partitions_singleapp.bin 0x10000 build/uart2udp.bin
