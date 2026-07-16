#pragma once

#include <Arduino.h>
#include <U8g2lib.h>

#include "meter_protocol.h"

namespace smart_meter {

inline void drawBootScreen(U8G2 &u8g2, const char *line1, const char *line2, const char *line3) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  if (line1 != nullptr) {
    u8g2.drawStr(0, 12, line1);
  }
  if (line2 != nullptr) {
    u8g2.drawStr(0, 28, line2);
  }
  if (line3 != nullptr) {
    u8g2.drawStr(0, 44, line3);
  }
  u8g2.sendBuffer();
}

inline void drawReadingScreen(U8G2 &u8g2, const Reading &reading, const char *wifiStatus, const char *mqttStatus, const char *meterStatus) {
  char line[32];

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);

  if (meterStatus != nullptr) {
    u8g2.drawStr(0, 0, meterStatus);
  }
  u8g2.drawStr(0, 12, "SMART METER");

  snprintf(line, sizeof(line), "V:%6.1f  A:%6.3f", reading.voltage, reading.current);
  u8g2.drawStr(0, 28, line);

  snprintf(line, sizeof(line), "P:%6.1f  E:%6.3f", reading.power, reading.energyKwh);
  u8g2.drawStr(0, 40, line);

  snprintf(line, sizeof(line), "F:%5.2f PF:%5.3f", reading.frequency, reading.powerFactor);
  u8g2.drawStr(0, 52, line);

  u8g2.setFont(u8g2_font_5x8_tf);
  if (wifiStatus != nullptr) {
    u8g2.drawStr(0, 63, wifiStatus);
  }
  if (mqttStatus != nullptr) {
    u8g2.drawStr(70, 63, mqttStatus);
  }

  u8g2.sendBuffer();
}

inline void drawStatusScreen(U8G2 &u8g2, const char *title, const char *line1, const char *line2) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  if (title != nullptr) {
    u8g2.drawStr(0, 12, title);
  }
  if (line1 != nullptr) {
    u8g2.drawStr(0, 32, line1);
  }
  if (line2 != nullptr) {
    u8g2.drawStr(0, 48, line2);
  }
  u8g2.sendBuffer();
}

}  // namespace smart_meter

