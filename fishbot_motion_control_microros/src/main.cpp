/**
 * @file main.cpp
 * @author fishros (fishros@foxmail.com)
 * @brief 主函数部分
 * @version V1.0.0
 * @date 2023-01-04
 * 
 * @copyright Copyright (c) fishros.com & fishros.org.cn 2023
 * 
 */
#include <Arduino.h>
#include "fishbot.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

void fishbot_loop_transport_task(void *param)
{
  setup_fishbot_transport();
  while (true)
  {
    loop_fishbot_transport();
  }
}

void setup()
{
  // Vince++ For issue - ESP32 Brownout detector was triggered
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  setup_fishbot();
  xTaskCreatePinnedToCore(fishbot_loop_transport_task, "transport_task", 10240, NULL, 1, NULL, 0);
}

void loop()
{
  delay(10);
  loop_fishbot_control();
}
