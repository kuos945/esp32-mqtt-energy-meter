#include <Arduino.h>
#include <PubSubClient.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <WiFi.h>

#include "meter_display.h"
#include "meter_network.h"
#include "meter_protocol.h"

constexpr char WIFI_SSID[] = "YOUR_WIFI_SSID";
constexpr char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";

constexpr char MQTT_HOST[] = "YOUR_MQTT_HOST";
constexpr uint16_t MQTT_PORT = 1883;
constexpr char MQTT_USER[] = "YOUR_MQTT_USERNAME";
constexpr char MQTT_PASSWORD[] = "YOUR_MQTT_PASSWORD";
constexpr char MQTT_TOPIC[] = "YOUR/MQTT/TOPIC";

constexpr char DEVICE_ID[] = "esp32-smart-meter";

constexpr uint8_t OLED_SDA = 21;
constexpr uint8_t OLED_SCL = 22;
constexpr uint8_t OLED_ADDR = 0x3C;
constexpr bool OLED_USE_SH1106 = false;

constexpr uint8_t RS485_RX_PIN = 16;
constexpr uint8_t RS485_TX_PIN = 17;
constexpr uint8_t RS485_DE_RE_PIN = 4;
constexpr uint32_t RS485_BAUD = 9600;

constexpr uint8_t METER_SLAVE_ID = 0x01;
constexpr uint16_t METER_START_REGISTER = 0x0000;
constexpr uint16_t METER_REGISTER_COUNT = 8;
constexpr uint32_t POLL_INTERVAL_MS = 2000UL;

#if OLED_USE_SH1106
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
#else
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
#endif

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

smart_meter::Reading meterReading;

char wifiStatus[32] = "WiFi boot";
char mqttStatus[32] = "MQTT boot";
char meterStatus[32] = "Meter boot";

uint32_t lastPollMs = 0;
uint32_t lastWifiRetryMs = 0;
uint32_t lastMqttRetryMs = 0;

void setHalfDuplexTransmit(bool transmit) {
  digitalWrite(RS485_DE_RE_PIN, transmit ? HIGH : LOW);
  delayMicroseconds(150);
}

bool queryMeter(smart_meter::Reading &reading) {
  const uint8_t request[] = {
      METER_SLAVE_ID,
      0x03,
      static_cast<uint8_t>(METER_START_REGISTER >> 8),
      static_cast<uint8_t>(METER_START_REGISTER & 0xFF),
      static_cast<uint8_t>(METER_REGISTER_COUNT >> 8),
      static_cast<uint8_t>(METER_REGISTER_COUNT & 0xFF),
      0x00,
      0x00};
  const uint16_t crc = smart_meter::crc16Modbus(request, 6);
  uint8_t frame[8];
  memcpy(frame, request, 8);
  frame[6] = static_cast<uint8_t>(crc & 0xFF);
  frame[7] = static_cast<uint8_t>(crc >> 8);

  while (Serial2.available() > 0) {
    Serial2.read();
  }

  setHalfDuplexTransmit(true);
  Serial2.write(frame, sizeof(frame));
  Serial2.flush();
  setHalfDuplexTransmit(false);

  return smart_meter::readHoldingResponse(Serial2, reading, METER_REGISTER_COUNT);
}

void drawAll() {
  smart_meter::drawReadingScreen(u8g2, meterReading, wifiStatus, mqttStatus, meterStatus);
}

void ensureNetwork() {
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWifiRetryMs >= 5000UL) {
      lastWifiRetryMs = millis();
      smart_meter::connectWiFi(WIFI_SSID, WIFI_PASSWORD, wifiStatus, sizeof(wifiStatus));
      drawAll();
    }
    return;
  }

  if (!mqttClient.connected()) {
    if (millis() - lastMqttRetryMs >= 5000UL) {
      lastMqttRetryMs = millis();
      const String clientId = String(DEVICE_ID) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
      smart_meter::connectMqtt(
          mqttClient,
          MQTT_HOST,
          MQTT_PORT,
          clientId.c_str(),
          MQTT_USER,
          MQTT_PASSWORD,
          mqttStatus,
          sizeof(mqttStatus));
      drawAll();
    }
    return;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.setI2CAddress(OLED_ADDR << 1);
  u8g2.begin();
  u8g2.setFontPosTop();
  u8g2.setFlipMode(1);
  smart_meter::drawBootScreen(u8g2, "Smart Meter", "Booting...", "Please wait");

  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  setHalfDuplexTransmit(false);

  smart_meter::connectWiFi(WIFI_SSID, WIFI_PASSWORD, wifiStatus, sizeof(wifiStatus));
  const String clientId = String(DEVICE_ID) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  mqttClient.setKeepAlive(30);
  mqttClient.setSocketTimeout(3);
  smart_meter::connectMqtt(
      mqttClient,
      MQTT_HOST,
      MQTT_PORT,
      clientId.c_str(),
      MQTT_USER,
      MQTT_PASSWORD,
      mqttStatus,
      sizeof(mqttStatus));

  smart_meter::drawStatusScreen(u8g2, "Smart Meter Ready", wifiStatus, mqttStatus);
}

void loop() {
  ensureNetwork();

  if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    mqttClient.loop();
  }

  if (millis() - lastPollMs >= POLL_INTERVAL_MS) {
    lastPollMs = millis();

    if (queryMeter(meterReading)) {
      smart_meter::setStatus(meterStatus, sizeof(meterStatus), "Meter ok");
      smart_meter::publishReading(mqttClient, MQTT_TOPIC, DEVICE_ID, meterReading, mqttStatus, sizeof(mqttStatus));
    } else {
      smart_meter::setStatus(meterStatus, sizeof(meterStatus), "Meter read fail");
    }

    drawAll();
  }

  delay(10);
}

