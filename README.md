## ROS2 二輪驅動車
### Hardware
1. 驅動主控板 - ESP32 montion control board
2. 雪達轉接 - ESP8266 UART 轉接Wi-Fi 或 USB
3. 雷達 - YDLIDAR-X2
4. 370帶編碼器減速馬達, 驅電機輪胎固定配件, 6P 2.0mm*10cm 馬達連接線
5. 65MM橡膠輪
6. 電池 2800ma 12V16A, 壓克力電池固定架, 12V1A充電器
7. SR04-超聲波
8. 0.96寸OLED顯示螢幕
9. 二驅壓克力半圓形底盤, 壓克力S雷達底板, 電壓轉換板, 10cm船型開關線
<table>
  <tboty>
    <tr>
      <td><img src="https://github.com/Kafkakav/fishbot2w_ros2/blob/main/docs/assets/367089_0.jpg" width="400" height="400"></td>
      <td><img src="https://github.com/Kafkakav/fishbot2w_ros2/blob/main/docs/assets/367088_0.jpg" width="400" height="400"></td>
    </tr>
    <tr>
      <td><img src="https://github.com/Kafkakav/fishbot2w_ros2/blob/main/docs/assets/367086_0.jpg" width="400" height="400"></td>
      <td><img src="https://github.com/Kafkakav/fishbot2w_ros2/blob/main/docs/assets/367085_0.jpg" width="400" height="400"></td>
    </tr>
    <tr>
      <td><img src="https://github.com/Kafkakav/fishbot2w_ros2/blob/main/docs/assets/367084_0.jpg" width="400" height="400"></td>
      <td><img src="https://github.com/Kafkakav/fishbot2w_ros2/blob/main/docs/assets/367083_0.jpg" width="400" height="400"></td>
    </tr>
  </tboty>
</table>

## FishBot 光達轉接板 + YDLIDAR-X2

### 光達轉接板 ESP-12E(ESP8266)
#### 光達轉接板功能: 
1. 接收來自YDLIDAR-X2的 serial Tx/Rx 資料
2. 把收到光達的資料轉送到USB UART(CH340)或remote WiFi (透過jumper設定)

#### 光達轉接板jumper設定: 
jumpers: EN,WIFI,USBT,FLASH,WIFI,USBR,FLASH
1. UART 傳送模式: => USBT, USBR
2. WIFI 傳送模式: => WIFI, WIFI
3. Flash 燒錄模式: => FLASH, FLASH

[原始教學網址](https://fishros.org.cn/forum/topic/940/fishbot%E6%95%99%E7%A8%8B-9-0-5-%E9%9B%B7%E8%BE%BE%E5%9B%BA%E4%BB%B6%E7%83%A7%E5%BD%95%E5%8F%8A%E9%85%8D%E7%BD%AE)

#### 更新輚接板firmware:
1.利用fishbot 配置助手(configuration)
2.利用esptool.py

#### 光達轉接板 Firmware
1. WiFi 模式
要透過網路傳送光達資料所以轉接板需要當作一個client的角色, 把收到的光達資料透過ESP-12E WIFI傳送出去, 到遠端的server.
遠端server是一個模擬UART的python程式, 它會虛擬一個/dev/ttyUSB的裝置, 讓ROS系統可以打開來使用

2. UART 模式
把光達的 serial Tx/Rx 線路透過jumper接到板子上的USB-Serial(USB UART). 任何支援USB-Serial的系統都可以透過這個port
收到光達的資料

### 光達轉接板 Firmware 
有三種模式:
1. 自己建制開發環境(hard)
2. 利用docker 容器 xingrz/esp8266-rtos-builder(easy)
3. 改寫code讓它符合 vscode的platform.io(hard++)

#### 1.自建開發環境
1. 安裝python 2.7 (有點老的版本, 如果沒有pyenv環境隔離, 再弄AI相關的專案肯定會衝突)
``` shell
#ubuntu ubuntu:20.04
apt-get update && \
    apt-get install -y gcc git wget make libncurses-dev flex bison gperf python && \
    wget https://bootstrap.pypa.io/pip/2.7/get-pip.py && python get-pip.py && rm -f get-pip.py &&\
    mkdir -p /esp
```

2. 安裝ESP8266 toolchains
``` shell
mkdir -p /esp

cd /esp && \
    wget https://dl.espressif.com/dl/xtensa-lx106-elf-gcc8_4_0-esp-2020r3-linux-amd64.tar.gz && \
    tar -xzf xtensa-lx106-elf-gcc8_4_0-esp-2020r3-linux-amd64.tar.gz && \
    rm -f xtensa-lx106-elf-gcc8_4_0-esp-2020r3-linux-amd64.tar.gz
```
[xtensa-lx106-elf-gcc8_4_0-esp-2020r3-win32.zip](https://dl.espressif.com/dl/xtensa-lx106-elf-gcc8_4_0-esp-2020r3-win32.zip)
[old version SDK(< 3.0), please use toolchain v4.8.5](https://dl.espressif.com/dl/xtensa-lx106-elf-win32-1.22.0-88-gde0bdc1-4.8.5.tar.gz)

3. 安裝ESP8266_RTOS_SDK
[參考](https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/get-started/index.html#get-esp8266-rtos-sdk)

```shell
cd ~/esp
git clone --recursive https://github.com/espressif/ESP8266_RTOS_SDK.git

# or
cd ~/esp
SDK_BRANCH= git clone \
    --recursive \
    --depth 1 \
    --branch ${SDK_BRANCH} \
    https://github.com/espressif/ESP8266_RTOS_SDK.git \
    /esp/ESP8266_RTOS_SDK

# Install the Required Python Packages
# IDF_PATH = ~/esp/ESP8266_RTOS_SDK
python -m pip install --user -r $IDF_PATH/requirements.txt
```

#### 2.利用docker容器 xingrz/esp8266-rtos-builder
1. xingrz/esp8266-rtos-builder
Dockerfile 內容如下:
``` yaml
FROM ubuntu:20.04

RUN apt-get update && \
    apt-get install -y gcc git wget make libncurses-dev flex bison gperf python && \
    wget https://bootstrap.pypa.io/pip/2.7/get-pip.py && python get-pip.py && rm -f get-pip.py &&\
    mkdir -p /esp

RUN cd /esp && \
    wget https://dl.espressif.com/dl/xtensa-lx106-elf-gcc8_4_0-esp-2020r3-linux-amd64.tar.gz && \
    tar -xzf xtensa-lx106-elf-gcc8_4_0-esp-2020r3-linux-amd64.tar.gz && \
    rm -f xtensa-lx106-elf-gcc8_4_0-esp-2020r3-linux-amd64.tar.gz

ARG SDK_BRANCH
RUN git clone \
    --recursive \
    --depth 1 \
    --branch ${SDK_BRANCH} \
    https://github.com/espressif/ESP8266_RTOS_SDK.git \
    /esp/ESP8266_RTOS_SDK

ENV PATH=${PATH}:/esp/xtensa-lx106-elf/bin
ENV IDF_PATH=/esp/ESP8266_RTOS_SDK

RUN python -m pip install --user -r ${IDF_PATH}/requirements.txt

VOLUME [ "/project" ]
WORKDIR /project

CMD [ "make" ]
```

2. Build, Merge and Flash
以下都是用docker 容器模式執行, 其中xingrz/esp8266-rtos-builder如上述.
``` shell
cd ~/fishbot-laser-control

##### make
docker run -it --rm --privileged -v=/dev:/dev  -v `pwd`:`pwd` -w `pwd` xingrz/esp8266-rtos-builder make 

##### merge: multiple bin files into one firmware.bin
docker run -it --rm --privileged -v=/dev:/dev  -v `pwd`:`pwd` -w `pwd` fishros2/fishbot-tool esptool.py \
--chip ESP8266 merge_bin -o fishbot_laser_control_v1.0.0.`date +%y%m%d`.bin \
--flash_mode dio \
--flash_size 4MB \
0x00 build/bootloader/bootloader.bin \
0x8000 build/partitions_singleapp.bin \
0x10000 build/uart2udp.bin

##### flash
docker run -it --rm --privileged -v=/dev:/dev  -v `pwd`:`pwd` -w `pwd` xingrz/esp8266-rtos-builder make flash
```

而 fishros2/fishbot-tool 是另一個容器, 專門用於fishbot教學燒錄開發板用
[fishbot_tool](https://github.com/fishros/fishbot_tool)
``` yaml
FROM ubuntu:jammy

RUN apt-get update && apt-get install -y \
    wget \
    fonts-wqy-zenhei \
    libgl1 \
    libegl1 \
    && apt-get clean && apt-get autoclean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

COPY "dist-for-docker/fishbot_tool_linux_amd64" /fishbot_tool
RUN chmod +x /fishbot_tool
ENTRYPOINT ["/fishbot_tool"]

# docker run -it --rm --privileged -v /dev:/dev  -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY fishbot-tool
```