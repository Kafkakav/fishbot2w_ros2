
![](./docs/images/1670950515258-0c1474f6-2d5a-4030-a1df-87bfdff78ba5-image-resized.png)

# How to build a new fireware
1. Use ubuntu Linux (Windows complained the path if microROS is too long)
2. Install vscode
3. Install vscode extension - platform.io
4. first time to build with vscode & platform.io (or run ./release.sh init)
5. after step 4, run release.sh to rebuild firmware
6. firmware file located at the path /bin
``` bash
cd fishbot_motion_control_microros
./release.sh
```

## Version file
``` C
# File: include/fishbot_config.h
#define VERSION_CODE "1.0.1p"
```

# FishBot运动控制程序MicroROS版
[original source](https://github.com/fishros/fishbot_motion_control_microros)
