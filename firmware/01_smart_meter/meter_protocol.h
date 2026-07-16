#pragma once

#include <Arduino.h>

namespace smart_meter {

struct Reading {
  bool valid = false;
  uint8_t slaveId = 0;
  uint8_t functionCode = 0;
  uint8_t exceptionCode = 0;
  uint16_t registerCount = 0;
  uint16_t raw[8] = {0};
  float voltage = 0.0f;
  float current = 0.0f;
  float power = 0.0f;
  float energyKwh = 0.0f;
  float frequency = 0.0f;
  float powerFactor = 0.0f;
  uint32_t frameMs = 0;
};

inline uint16_t crc16Modbus(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

inline size_t buildReadHoldingRequest(uint8_t slaveId, uint16_t startRegister, uint16_t registerCount, uint8_t *out, size_t outSize) {
  if (out == nullptr || outSize < 8) {
    return 0;
  }

  out[0] = slaveId;
  out[1] = 0x03;
  out[2] = static_cast<uint8_t>(startRegister >> 8);
  out[3] = static_cast<uint8_t>(startRegister & 0xFF);
  out[4] = static_cast<uint8_t>(registerCount >> 8);
  out[5] = static_cast<uint8_t>(registerCount & 0xFF);

  const uint16_t crc = crc16Modbus(out, 6);
  out[6] = static_cast<uint8_t>(crc & 0xFF);
  out[7] = static_cast<uint8_t>(crc >> 8);
  return 8;
}

inline bool readExact(Stream &port, uint8_t *buffer, size_t length, uint32_t timeoutMs) {
  if (buffer == nullptr) {
    return false;
  }

  size_t index = 0;
  uint32_t lastActivityMs = millis();

  while (index < length) {
    while (port.available() > 0 && index < length) {
      const int value = port.read();
      if (value < 0) {
        continue;
      }

      buffer[index++] = static_cast<uint8_t>(value);
      lastActivityMs = millis();
    }

    if (index >= length) {
      return true;
    }

    if (millis() - lastActivityMs > timeoutMs) {
      return false;
    }

    delay(1);
  }

  return true;
}

inline void applyDefaultScales(Reading &reading) {
  if (reading.registerCount > 0) {
    reading.voltage = reading.raw[0] * 0.1f;
  }
  if (reading.registerCount > 1) {
    reading.current = reading.raw[1] * 0.001f;
  }
  if (reading.registerCount > 2) {
    reading.power = reading.raw[2] * 0.1f;
  }
  if (reading.registerCount > 3) {
    reading.energyKwh = reading.raw[3] * 0.001f;
  }
  if (reading.registerCount > 4) {
    reading.frequency = reading.raw[4] * 0.01f;
  }
  if (reading.registerCount > 5) {
    reading.powerFactor = reading.raw[5] * 0.001f;
  }
}

inline bool parseReadHoldingResponse(const uint8_t *frame, size_t length, Reading &reading, uint16_t expectedRegisterCount) {
  if (frame == nullptr || length < 5) {
    return false;
  }

  const uint8_t slaveId = frame[0];
  const uint8_t functionCode = frame[1];

  if (functionCode & 0x80) {
    if (length != 5) {
      return false;
    }

    const uint16_t crc = crc16Modbus(frame, length - 2);
    const uint16_t receivedCrc = static_cast<uint16_t>(frame[length - 2]) |
                                (static_cast<uint16_t>(frame[length - 1]) << 8);
    if (crc != receivedCrc) {
      return false;
    }

    reading = Reading{};
    reading.slaveId = slaveId;
    reading.functionCode = functionCode;
    reading.exceptionCode = frame[2];
    reading.frameMs = millis();
    return false;
  }

  if (functionCode != 0x03) {
    return false;
  }

  const uint8_t byteCount = frame[2];
  const size_t expectedLength = static_cast<size_t>(byteCount) + 5;

  if (length != expectedLength) {
    return false;
  }

  const uint16_t crc = crc16Modbus(frame, length - 2);
  const uint16_t receivedCrc = static_cast<uint16_t>(frame[length - 2]) |
                              (static_cast<uint16_t>(frame[length - 1]) << 8);
  if (crc != receivedCrc) {
    return false;
  }

  reading = Reading{};
  reading.slaveId = slaveId;
  reading.functionCode = functionCode;
  reading.registerCount = expectedRegisterCount;
  reading.frameMs = millis();

  if (byteCount < expectedRegisterCount * 2) {
    return false;
  }

  const size_t registerLimit = min<size_t>(expectedRegisterCount, sizeof(reading.raw) / sizeof(reading.raw[0]));
  for (size_t i = 0; i < registerLimit; ++i) {
    const size_t offset = 3 + i * 2;
    reading.raw[i] = (static_cast<uint16_t>(frame[offset]) << 8) |
                     static_cast<uint16_t>(frame[offset + 1]);
  }

  applyDefaultScales(reading);
  reading.valid = true;
  return true;
}

inline bool requestAndRead(Stream &port, uint8_t slaveId, uint16_t startRegister, uint16_t registerCount, Reading &reading, uint32_t timeoutMs = 600) {
  uint8_t request[8];
  const size_t requestLength = buildReadHoldingRequest(slaveId, startRegister, registerCount, request, sizeof(request));
  if (requestLength == 0) {
    return false;
  }

  while (port.available() > 0) {
    port.read();
  }

  port.write(request, requestLength);
  port.flush();

  uint8_t header[3];
  if (!readExact(port, header, sizeof(header), timeoutMs)) {
    return false;
  }

  const uint8_t byteCount = header[2];
  uint8_t frame[3 + 16 + 2] = {0};
  frame[0] = header[0];
  frame[1] = header[1];
  frame[2] = header[2];

  if (byteCount > 16) {
    return false;
  }

  if (!readExact(port, frame + 3, static_cast<size_t>(byteCount) + 2, timeoutMs)) {
    return false;
  }

  const size_t frameLength = static_cast<size_t>(byteCount) + 5;
  return parseReadHoldingResponse(frame, frameLength, reading, registerCount);
}

inline bool readHoldingResponse(Stream &port, Reading &reading, uint16_t expectedRegisterCount, uint32_t timeoutMs = 600) {
  uint8_t header[3];
  if (!readExact(port, header, sizeof(header), timeoutMs)) {
    return false;
  }

  const uint8_t byteCount = header[2];
  if (byteCount > 16) {
    return false;
  }

  uint8_t frame[3 + 16 + 2] = {0};
  frame[0] = header[0];
  frame[1] = header[1];
  frame[2] = header[2];

  if (!readExact(port, frame + 3, static_cast<size_t>(byteCount) + 2, timeoutMs)) {
    return false;
  }

  const size_t frameLength = static_cast<size_t>(byteCount) + 5;
  return parseReadHoldingResponse(frame, frameLength, reading, expectedRegisterCount);
}

}  // namespace smart_meter

