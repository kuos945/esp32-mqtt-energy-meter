#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

constexpr uint8_t OLED_SDA = 21;
constexpr uint8_t OLED_SCL = 22;
constexpr uint8_t OLED_ADDR = 0x3C;
constexpr int OLED_RESET = -1;
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;

constexpr int METER_RX_PIN = 16;
constexpr int METER_TX_PIN = 17;
constexpr uint32_t READ_INTERVAL_MS = 2000UL;
constexpr size_t PZEM_FRAME_SIZE = 25;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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

bool readPzemFrame(uint8_t *frame, size_t frameSize);
bool decodePzemFrame(const uint8_t *frame, size_t frameSize, MeterReading &reading);
void drawMeterScreen(const MeterReading &reading, const char *statusLine);

static uint16_t readU16BE(const uint8_t *frame, size_t index) {
  return static_cast<uint16_t>((static_cast<uint16_t>(frame[index]) << 8) |
                               static_cast<uint16_t>(frame[index + 1]));
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

void drawMeterScreen(const MeterReading &reading, const char *statusLine) {
  char line2[32];
  char line3[32];
  char line4[32];

  if (reading.valid) {
    snprintf(line2, sizeof(line2), "V:%5.1fV I:%6.3fA", reading.voltage, reading.current);
    snprintf(line3, sizeof(line3), "P:%5.1fW E:%7.3fkWh", reading.power, reading.energy);
    snprintf(line4, sizeof(line4), "F:%5.1fHz PF:%4.2f", reading.frequency, reading.pf);
  } else {
    snprintf(line2, sizeof(line2), "Waiting meter data");
    snprintf(line3, sizeof(line3), "Check serial wiring");
    snprintf(line4, sizeof(line4), "%s", statusLine);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.println(F("ENERGY METER"));
  display.setCursor(0, 16);
  display.println(line2);
  display.setCursor(0, 32);
  display.println(line3);
  display.setCursor(0, 48);
  display.println(line4);
  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(300);

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
  Serial.println(F("Energy meter monitor ready"));
  drawMeterScreen(lastReading, "Booting...");
}

void loop() {
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
      drawMeterScreen(lastReading, "OK");
    } else {
      lastReading.valid = false;
      Serial.println(F("Meter read failed"));
      drawMeterScreen(lastReading, "No meter data");
    }
  }

  delay(20);
}

