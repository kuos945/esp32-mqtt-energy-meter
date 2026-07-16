#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <SimpleDHT.h>

constexpr uint8_t OLED_SDA = 21;
constexpr uint8_t OLED_SCL = 22;
constexpr uint8_t OLED_ADDR = 0x3C;
constexpr int OLED_RESET = -1;
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;

constexpr int METER_RX_PIN = 16;
constexpr int METER_TX_PIN = 17;
constexpr int RELAY_PIN = 18;
constexpr int SG90_PIN = 5;
constexpr int PHOTO_PIN = 33;
constexpr int DHT_PIN = 14;
constexpr int SG90_LEDC_CHANNEL = 0;
constexpr int SG90_LEDC_RESOLUTION = 16;
constexpr int SG90_LEDC_FREQUENCY = 50;
constexpr int ADC_MAX = 4095;

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
SimpleDHT11 dht(DHT_PIN);

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
uint32_t lastDisplaySwitchMs = 0;
uint8_t currentDisplayPage = 0;

int currentTemperatureC = -1;
int currentHumidityPercent = -1;
int currentLightPercent = -1;
bool currentSensorOk = false;

bool relayState = true;
int sg90CommandAngle = 0;
int sg90ActualAngle = 0;

bool readPzemFrame(uint8_t *frame, size_t frameSize);
bool decodePzemFrame(const uint8_t *frame, size_t frameSize, MeterReading &reading);
void drawMeterScreen(const MeterReading &reading);
void drawSensorScreen(int temperatureC, int humidityPercent, int lightPercent, bool sensorOk);
void renderDisplay();
void ensureWiFiConnected();
void ensureMqttConnected();
bool publishMeterJson(const MeterReading &reading, int temperatureC, int humidityPercent, int lightPercent);
void mqttCallback(char *topic, byte *payload, unsigned int length);
void formatTrimmedFloat(char *buffer, size_t bufferSize, float value, uint8_t digits);
void setRelay(bool on);
void initServo();
void writeServoAngle(int inputAngle);
void applyServoCommand(int inputAngle);
int readBrightnessPercent();
bool readTemperatureHumidity(int &temperatureC, int &humidityPercent);

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

int readBrightnessPercent() {
  const int raw = analogRead(PHOTO_PIN);
  const int percent = map(raw, 0, ADC_MAX, 0, 100);
  return constrain(percent, 0, 100);
}

bool readTemperatureHumidity(int &temperatureC, int &humidityPercent) {
  float temperature = 0.0f;
  float humidity = 0.0f;
  const int err = dht.read2(&temperature, &humidity, NULL);
  if (err != SimpleDHTErrSuccess) {
    Serial.printf("DHT11 read failed: %d\n", err);
    return false;
  }

  temperatureC = static_cast<int>(roundf(temperature));
  humidityPercent = static_cast<int>(roundf(humidity));
  humidityPercent = constrain(humidityPercent, 0, 100);
  return true;
}

void setRelay(bool on) {
  relayState = on;
  digitalWrite(RELAY_PIN, on ? LOW : HIGH);
  Serial.printf("Relay %s\n", on ? "ON" : "OFF");
}

void initServo() {
  ledcSetup(SG90_LEDC_CHANNEL, SG90_LEDC_FREQUENCY, SG90_LEDC_RESOLUTION);
  ledcAttachPin(SG90_PIN, SG90_LEDC_CHANNEL);
  applyServoCommand(0);
}

void writeServoAngle(int inputAngle) {
  const int clampedAngle = constrain(inputAngle, 0, 180);
  const int pulseUs = map(clampedAngle, 0, 180, 500, 2400);
  const uint32_t maxDuty = (1UL << SG90_LEDC_RESOLUTION) - 1UL;
  const uint32_t duty = static_cast<uint32_t>((static_cast<uint64_t>(pulseUs) * maxDuty) / 20000UL);
  ledcWrite(SG90_LEDC_CHANNEL, duty);
  sg90ActualAngle = clampedAngle;
  Serial.printf("SG90 actual=%d\n", sg90ActualAngle);
}

void applyServoCommand(int inputAngle) {
  sg90CommandAngle = constrain(inputAngle, 0, 180);
  Serial.printf("SG90 cmd=%d actual=%d\n", sg90CommandAngle, sg90CommandAngle);
  writeServoAngle(sg90CommandAngle);
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

bool publishMeterJson(const MeterReading &reading, int temperatureC, int humidityPercent, int lightPercent) {
  if (!mqttClient.connected()) {
    return false;
  }

  char vBuf[16];
  char iBuf[16];
  char wBuf[16];
  char kWhBuf[16];
  char fBuf[16];
  char pfBuf[16];
  char tempBuf[12];
  char humiBuf[12];
  char lightBuf[12];

  formatTrimmedFloat(vBuf, sizeof(vBuf), reading.voltage, 1);
  formatTrimmedFloat(iBuf, sizeof(iBuf), reading.current, 3);
  formatTrimmedFloat(wBuf, sizeof(wBuf), reading.power, 1);
  formatTrimmedFloat(kWhBuf, sizeof(kWhBuf), reading.energy, 3);
  formatTrimmedFloat(fBuf, sizeof(fBuf), reading.frequency, 1);
  formatTrimmedFloat(pfBuf, sizeof(pfBuf), reading.pf, 2);
  snprintf(tempBuf, sizeof(tempBuf), "%d", temperatureC);
  snprintf(humiBuf, sizeof(humiBuf), "%d", humidityPercent);
  snprintf(lightBuf, sizeof(lightBuf), "%d", lightPercent);

  char payload[192];
  snprintf(payload, sizeof(payload),
           "{\"V\":%s,\"I\":%s,\"W\":%s,\"kWh\":%s,\"F\":%s,\"PF\":%s,\"temp\":%s,\"humi\":%s,\"light\":%s}",
           vBuf, iBuf, wBuf, kWhBuf, fBuf, pfBuf, tempBuf, humiBuf, lightBuf);

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

  if (doc["RELAY"].is<const char *>()) {
    const char *relayCmd = doc["RELAY"];
    if (strcmp(relayCmd, "ON") == 0) {
      setRelay(true);
    } else if (strcmp(relayCmd, "OFF") == 0) {
      setRelay(false);
    }
  }

  if (doc["SG90"].is<int>()) {
    const int rawAngle = doc["SG90"].as<int>();
    if (rawAngle >= 0 && rawAngle <= 180) {
      applyServoCommand(rawAngle);
    } else {
      Serial.printf("SG90 ignored out of range: %d\n", rawAngle);
    }
  }
}

void drawMeterScreen(const MeterReading &reading) {
  char vLine[16];
  char iLine[16];
  char pLine[16];
  char eLine[24];
  char fLine[16];
  char pfLine[16];
  char energyBuf[12];

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

void drawSensorScreen(int temperatureC, int humidityPercent, int lightPercent, bool sensorOk) {
  char tempLine[24];
  char humiLine[24];
  char lightLine[24];

  if (sensorOk) {
    snprintf(tempLine, sizeof(tempLine), "temp:%d C", temperatureC);
    snprintf(humiLine, sizeof(humiLine), "humi:%d %%", humidityPercent);
  } else {
    snprintf(tempLine, sizeof(tempLine), "temp:-- C");
    snprintf(humiLine, sizeof(humiLine), "humi:-- %%");
  }
  snprintf(lightLine, sizeof(lightLine), "light:%d %%", lightPercent);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.println(F("SENSOR STATUS"));
  display.setCursor(0, 16);
  display.print(tempLine);
  display.setCursor(0, 32);
  display.print(humiLine);
  display.setCursor(0, 48);
  display.print(lightLine);
  display.display();
}

void renderDisplay() {
  if (currentDisplayPage == 0) {
    drawMeterScreen(lastReading);
  } else {
    drawSensorScreen(currentTemperatureC, currentHumidityPercent, currentLightPercent, currentSensorOk);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(PHOTO_PIN, INPUT);
  analogReadResolution(12);
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
  initServo();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setBufferSize(256);
  mqttClient.setCallback(mqttCallback);

  Serial.println(F("Energy meter relay SG90 MQTT monitor ready"));
  renderDisplay();
}

void loop() {
  ensureWiFiConnected();
  ensureMqttConnected();

  if (mqttClient.connected()) {
    mqttClient.loop();
  }

  const uint32_t now = millis();
  if (lastDisplaySwitchMs == 0) {
    lastDisplaySwitchMs = now;
  } else if (now - lastDisplaySwitchMs >= 5000UL) {
    lastDisplaySwitchMs = now;
    currentDisplayPage ^= 1U;
    renderDisplay();
  }

  if (lastReadMs == 0 || now - lastReadMs >= READ_INTERVAL_MS) {
    lastReadMs = now;

    uint8_t frame[PZEM_FRAME_SIZE] = {0};
    const bool gotFrame = readPzemFrame(frame, sizeof(frame));
    const bool ok = gotFrame && decodePzemFrame(frame, sizeof(frame), lastReading);
    const int lightPercent = readBrightnessPercent();
    int temperatureC = -1;
    int humidityPercent = -1;
    const bool dhtOk = readTemperatureHumidity(temperatureC, humidityPercent);

    currentLightPercent = lightPercent;
    currentSensorOk = dhtOk;
    currentTemperatureC = dhtOk ? temperatureC : -1;
    currentHumidityPercent = dhtOk ? humidityPercent : -1;

    if (ok) {
      Serial.printf("V=%.1f V, I=%.3f A, P=%.1f W, E=%.3f kWh, F=%.1f Hz, PF=%.2f\n",
                    lastReading.voltage,
                    lastReading.current,
                    lastReading.power,
                    lastReading.energy,
                    lastReading.frequency,
                    lastReading.pf);
      if (!dhtOk) {
        temperatureC = -1;
        humidityPercent = -1;
      }
      publishMeterJson(lastReading, temperatureC, humidityPercent, lightPercent);
    } else {
      lastReading.valid = false;
      Serial.println(F("Meter read failed"));
    }

    renderDisplay();
  }

  delay(20);
}

