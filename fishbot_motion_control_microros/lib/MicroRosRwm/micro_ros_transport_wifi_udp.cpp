#include <Arduino.h>

#include <micro_ros_platformio.h>

#include <WiFi.h>
#include <WiFiUdp.h>

#include <uxr/client/util/time.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>
#include "micro_ros_transport_wifi_udp.h"
extern "C"
{

  static WiFiUDP udp_client;

  bool platformio_transport_open_wifi_udp(struct uxrCustomTransport *transport)
  {
    struct micro_ros_agent_locator *locator = (struct micro_ros_agent_locator *)transport->args;
    
    Serial.print("XXXXX_open_udp: "); Serial.println(locator->port);

    return true == udp_client.begin(locator->port);
  }

  bool platformio_transport_close_wifi_udp(struct uxrCustomTransport *transport)
  {
    Serial.println("XXXXX_close_udp");
    udp_client.stop();
    return true;
  }

  size_t platformio_transport_write_wifi_udp(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *errcode)
  {
    (void)errcode;
    struct micro_ros_agent_locator *locator = (struct micro_ros_agent_locator *)transport->args;

    size_t sent = 0;
    if (true == udp_client.beginPacket(locator->address, locator->port))
    {
      sent = udp_client.write(buf, len);
      sent = true == udp_client.endPacket() ? sent : 0;
    }

    udp_client.flush();
    return sent;
  }

  size_t platformio_transport_read_wifi_udp(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
  {
    (void)errcode;

    int64_t start_time = uxr_millis();

    while ((uxr_millis() - start_time) < ((int64_t)timeout) && udp_client.parsePacket() == 0)
    {
      delay(1);
    }

    size_t available = 0;
    if (udp_client.available())
    {
      available = udp_client.read(buf, len);
    }

    return (available > 0) ? available : 0;
  }
}