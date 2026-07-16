#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>

constexpr uint8_t OLED_SDA = 21;
constexpr uint8_t OLED_SCL = 22;
constexpr uint8_t OLED_ADDR = 0x3C;
constexpr int OLED_RESET = -1;
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;

constexpr int METER_RX_PIN = 16;
constexpr int METER_TX_PIN = 17;
constexpr int RELAY_PIN = 18;
constexpr uint32_t READ_INTERVAL_MS = 2000UL;
constexpr size_t PZEM_FRAME_SIZE = 25;

constexpr char WIFI_SSID[] = "YOUR_WIFI_SSID";
constexpr char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";

constexpr char MQTT_HOST[] = "YOUR_MQTT_HOST";
constexpr uint16_t MQTT_PORT = 1883;
constexpr char MQTT_USERNAME[] = "YOUR_MQTT_USERNAME";
constexpr char MQTT_PASSWORD[] = "YOUR_MQTT_PASSWORD";
constexpr char MQTT_TOPIC[] = "YOUR/MQTT/DATA_TOPIC";
constexpr char MQTT_SUB_TOPIC[] = "YOUR/MQTT/CTRL_TOPIC";

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

struct MeterReading {
  float voltage;
  float current;
  float power;
  float energy;
  float frequency;
  float pf;
  bool valid;
};

MeterReading lastReading{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
uint32_t lastReadMs = 0;
uint32_t lastWiFiAttemptMs = 0;
uint32_t lastMqttAttemptMs = 0;

bool relayState = true;

bool readPzemFrame(uint8_t *frame, size_t frameSize);
bool decodePzemFrame(const uint8_t *frame, size_t frameSize, MeterReading &reading);
void drawMeterScreen(const MeterReading &reading);
void ensureWiFiConnected();
void ensureMqttConnected();
bool publishMeterJson(const MeterReading &reading);
void mqttCallback(char *topic, byte *payload, unsigned int length);
void formatTrimmedFloat(char *buffer, size_t bufferSize, float value, uint8_t digits);
void setRelay(bool on);

static uint16_t readU16BE(const uint8_t *frame, size_t index) {
  return static_cast<uint16_t>((static_cast<uint16_t>(frame[index]) << 8) |
                               static_cast<uint16_t>(frame[index + 1]));
}

void formatTrimmedFloat(char *buffer, size_t bufferSize, float value, uint8_t digits) {
  char temp[24];
  snprintf(temp, sizeof(temp), "%.*f", digits, value);

  char *dot = strchr(temp, '.');
  if (dot != nullptr) {
    char *end = temp + strlen(temp) - 1;
    while (end > dot && *end == '0') {
      *end-- = '\0';
    }
    if (end == dot) {
      *end = '\0';
    }
  }

  snprintf(buffer, bufferSize, "%s", temp);
}

void setRelay(bool on) {
  relayState = on;
  digitalWrite(RELAY_PIN, on ? LOW : HIGH);
  Serial.printf("Relay %s\n", on ? "ON" : "OFF");
}

bool readPzemFrame(uint8_t *frame, size_t frameSize) {
  while (Serial2.available() > 0) {
    Serial2.read();
  }

  const uint8_t request[] = {0xF8, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x64, 0x64};
  Serial2.write(request, sizeof(request));
  Serial2.flush();

  const uint32_t start = millis();
  size_t count = 0;
  while (millis() - start < 500UL) {
    while (Serial2.available() > 0 && count < frameSize) {
      frame[count++] = static_cast<uint8_t>(Serial2.read());
    }
    if (count >= frameSize) {
      break;
    }
    delay(5);
  }

  return count >= frameSize;
}

bool decodePzemFrame(const uint8_t *frame, size_t frameSize, MeterReading &reading) {
  if (frameSize < PZEM_FRAME_SIZE) {
    return false;
  }

  if (frame[0] != 0xF8 || frame[1] != 0x04 || frame[2] != 0x14) {
    return false;
  }

  const uint16_t rawVoltage = readU16BE(frame, 3);
  const uint16_t rawCurrent = readU16BE(frame, 5);
  const uint16_t rawPower = readU16BE(frame, 9);
  const uint16_t rawEnergy = readU16BE(frame, 13);
  const uint16_t rawFrequency = readU16BE(frame, 17);
  const uint16_t rawPf = readU16BE(frame, 19);

  reading.voltage = rawVoltage * 0.1f;
  reading.current = rawCurrent * 0.001f;
  reading.power = rawPower * 0.1f;
  reading.energy = rawEnergy * 0.001f;
  reading.frequency = rawFrequency * 0.1f;
  reading.pf = rawPf * 0.01f;
  if (reading.pf < 0.0f) {
    reading.pf = 0.0f;
  } else if (reading.pf > 1.0f) {
    reading.pf = 1.0f;
  }
  reading.valid = true;
  return true;
}

void ensureWiFiConnected() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  const uint32_t now = millis();
  if (lastWiFiAttemptMs != 0 && now - lastWiFiAttemptMs < 5000UL) {
    return;
  }
  lastWiFiAttemptMs = now;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println(F("WiFi connecting"));
}

void ensureMqttConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (mqttClient.connected()) {
    return;
  }

  const uint32_t now = millis();
  if (lastMqttAttemptMs != 0 && now - lastMqttAttemptMs < 5000UL) {
    return;
  }
  lastMqttAttemptMs = now;

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setBufferSize(256);
  mqttClient.setCallback(mqttCallback);

  String clientId = "esp32-meter-";
  clientId += String((uint32_t)ESP.getEfuseMac(), HEX);

  Serial.println(F("MQTT connecting"));
  if (mqttClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
    Serial.println(F("MQTT connected"));
    mqttClient.subscribe(MQTT_SUB_TOPIC);
  } else {
    Serial.printf("MQTT connect failed, state=%d\n", mqttClient.state());
  }
}

bool publishMeterJson(const MeterReading &reading) {
  if (!mqttClient.connected()) {
    return false;
  }

  char vBuf[16];
  char iBuf[16];
  char wBuf[16];
  char kWhBuf[16];
  char fBuf[16];
  char pfBuf[16];

  formatTrimmedFloat(vBuf, sizeof(vBuf), reading.voltage, 1);
  formatTrimmedFloat(iBuf, sizeof(iBuf), reading.current, 3);
  formatTrimmedFloat(wBuf, sizeof(wBuf), reading.power, 1);
  formatTrimmedFloat(kWhBuf, sizeof(kWhBuf), reading.energy, 3);
  formatTrimmedFloat(fBuf, sizeof(fBuf), reading.frequency, 1);
  formatTrimmedFloat(pfBuf, sizeof(pfBuf), reading.pf, 2);

  char payload[128];
  snprintf(payload, sizeof(payload),
           "{\"V\":%s,\"I\":%s,\"W\":%s,\"kWh\":%s,\"F\":%s,\"PF\":%s}",
           vBuf, iBuf, wBuf, kWhBuf, fBuf, pfBuf);

  const bool ok = mqttClient.publish(MQTT_TOPIC, payload, false);
  if (ok) {
    Serial.print(F("MQTT pub: "));
    Serial.println(payload);
  } else {
    Serial.println(F("MQTT publish failed"));
  }

  return ok;
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  if (strcmp(topic, MQTT_SUB_TOPIC) != 0) {
    return;
  }

  String payloadText;
  payloadText.reserve(length);
  for (unsigned int i = 0; i < length; ++i) {
    payloadText += static_cast<char>(payload[i]);
  }

  Serial.print(F("MQTT RX topic="));
  Serial.print(topic);
  Serial.print(F(" payload="));
  Serial.println(payloadText);

  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.print(F("MQTT JSON parse failed: "));
    Serial.println(err.c_str());
    return;
  }

  if (!doc["RELAY"].is<const char *>()) {
    return;
  }

  const char *relayCmd = doc["RELAY"];
  if (strcmp(relayCmd, "ON") == 0) {
    setRelay(true);
  } else if (strcmp(relayCmd, "OFF") == 0) {
    setRelay(false);
  }
}

void drawMeterScreen(const MeterReading &reading) {
  char vLine[16];
  char iLine[16];
  char pLine[16];
  char eLine[16];
  char fLine[16];
  char pfLine[16];
  char energyBuf[16];

  if (reading.valid) {
    snprintf(vLine, sizeof(vLine), "V:%5.1fV", reading.voltage);
    snprintf(iLine, sizeof(iLine), "I:%6.3fA", reading.current);
    snprintf(pLine, sizeof(pLine), "P:%5.1fW", reading.power);
    formatTrimmedFloat(energyBuf, sizeof(energyBuf), reading.energy, 3);
    snprintf(eLine, sizeof(eLine), "E:%skWh", energyBuf);
    snprintf(fLine, sizeof(fLine), "F:%5.1fHz", reading.frequency);
    snprintf(pfLine, sizeof(pfLine), "PF:%4.2f", reading.pf);
  } else {
    snprintf(vLine, sizeof(vLine), "V:--.-V");
    snprintf(iLine, sizeof(iLine), "I:--.---A");
    snprintf(pLine, sizeof(pLine), "P:--.-W");
    snprintf(eLine, sizeof(eLine), "E:--.---kWh");
    snprintf(fLine, sizeof(fLine), "F:--.-Hz");
    snprintf(pfLine, sizeof(pfLine), "PF:--.--");
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.println(F("ENERGY METER"));

  display.setCursor(0, 16);
  display.print(vLine);
  display.setCursor(66, 16);
  display.print(iLine);

  display.setCursor(0, 32);
  display.print(pLine);
  display.setCursor(66, 32);
  display.print(eLine);

  display.setCursor(0, 48);
  display.print(fLine);
  display.setCursor(66, 48);
  display.print(pfLine);

  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(RELAY_PIN, OUTPUT);
  setRelay(true);

  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 init failed"));
    while (true) {
      delay(1000);
    }
  }

  display.setRotation(2);
  display.clearDisplay();
  display.display();

  Serial2.begin(9600, SERIAL_8N1, METER_RX_PIN, METER_TX_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setBufferSize(256);
  mqttClient.setCallback(mqttCallback);

  Serial.println(F("Energy meter relay MQTT monitor ready"));
  drawMeterScreen(lastReading);
}

void loop() {
  ensureWiFiConnected();
  ensureMqttConnected();

  if (mqttClient.connected()) {
    mqttClient.loop();
  }

  const uint32_t now = millis();
  if (lastReadMs == 0 || now - lastReadMs >= READ_INTERVAL_MS) {
    lastReadMs = now;

    uint8_t frame[PZEM_FRAME_SIZE] = {0};
    const bool gotFrame = readPzemFrame(frame, sizeof(frame));
    const bool ok = gotFrame && decodePzemFrame(frame, sizeof(frame), lastReading);

    if (ok) {
      Serial.printf("V=%.1f V, I=%.3f A, P=%.1f W, E=%.3f kWh, F=%.1f Hz, PF=%.2f\n",
                    lastReading.voltage,
                    lastReading.current,
                    lastReading.power,
                    lastReading.energy,
                    lastReading.frequency,
                    lastReading.pf);
      publishMeterJson(lastReading);
    } else {
      lastReading.valid = false;
      Serial.println(F("Meter read failed"));
    }

    drawMeterScreen(lastReading);
  }

  delay(20);
}

