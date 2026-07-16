#pragma once

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include "meter_protocol.h"

namespace smart_meter {

inline void setStatus(char *buffer, size_t bufferSize, const char *text) {
  if (buffer == nullptr || bufferSize == 0) {
    return;
  }

  if (text == nullptr) {
    buffer[0] = '\0';
    return;
  }

  size_t i = 0;
  for (; i + 1 < bufferSize && text[i] != '\0'; ++i) {
    buffer[i] = text[i];
  }
  buffer[i] = '\0';
}

inline bool connectWiFi(const char *ssid, const char *password, char *statusBuffer, size_t statusBufferSize, uint32_t timeoutMs = 15000) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    char line[64];
    snprintf(line, sizeof(line), "WiFi %s", WiFi.localIP().toString().c_str());
    setStatus(statusBuffer, statusBufferSize, line);
    return true;
  }

  setStatus(statusBuffer, statusBufferSize, "WiFi timeout");
  return false;
}

inline bool connectMqtt(PubSubClient &client, const char *host, uint16_t port, const char *clientId, const char *username, const char *password, char *statusBuffer, size_t statusBufferSize) {
  client.setServer(host, port);
  client.setBufferSize(512);

  if (WiFi.status() != WL_CONNECTED) {
    setStatus(statusBuffer, statusBufferSize, "MQTT wait WiFi");
    return false;
  }

  if (client.connected()) {
    setStatus(statusBuffer, statusBufferSize, "MQTT ready");
    return true;
  }

  const bool ok = client.connect(clientId, username, password);
  if (ok) {
    setStatus(statusBuffer, statusBufferSize, "MQTT connected");
  } else {
    char line[48];
    snprintf(line, sizeof(line), "MQTT err %d", client.state());
    setStatus(statusBuffer, statusBufferSize, line);
  }
  return ok;
}

inline bool publishReading(PubSubClient &client, const char *topic, const char *deviceId, const Reading &reading, char *statusBuffer, size_t statusBufferSize) {
  if (!client.connected()) {
    setStatus(statusBuffer, statusBufferSize, "MQTT disconnected");
    return false;
  }

  char payload[320];
  snprintf(
      payload,
      sizeof(payload),
      "{\"device\":\"%s\",\"valid\":%s,\"V\":%.1f,\"A\":%.3f,\"W\":%.1f,\"kWh\":%.3f,\"Hz\":%.2f,\"PF\":%.3f,\"ms\":%lu}",
      deviceId != nullptr ? deviceId : "esp32",
      reading.valid ? "true" : "false",
      reading.voltage,
      reading.current,
      reading.power,
      reading.energyKwh,
      reading.frequency,
      reading.powerFactor,
      static_cast<unsigned long>(reading.frameMs));

  const bool ok = client.publish(topic, payload);
  if (ok) {
    setStatus(statusBuffer, statusBufferSize, "MQTT publish ok");
  } else {
    setStatus(statusBuffer, statusBufferSize, "MQTT publish fail");
  }
  return ok;
}

inline bool ensureConnectivity(
    const char *ssid,
    const char *wifiPassword,
    PubSubClient &mqttClient,
    const char *mqttHost,
    uint16_t mqttPort,
    const char *mqttClientId,
    const char *mqttUser,
    const char *mqttPassword,
    char *wifiStatus,
    size_t wifiStatusSize,
    char *mqttStatus,
    size_t mqttStatusSize) {
  const bool wifiOk = WiFi.status() == WL_CONNECTED || connectWiFi(ssid, wifiPassword, wifiStatus, wifiStatusSize);
  const bool mqttOk = wifiOk && (mqttClient.connected() || connectMqtt(mqttClient, mqttHost, mqttPort, mqttClientId, mqttUser, mqttPassword, mqttStatus, mqttStatusSize));

  if (!wifiOk && WiFi.status() != WL_CONNECTED) {
    setStatus(mqttStatus, mqttStatusSize, "MQTT wait WiFi");
  } else if (wifiOk && !mqttOk) {
    setStatus(mqttStatus, mqttStatusSize, "MQTT retry");
  }

  return wifiOk && mqttOk;
}

}  // namespace smart_meter

