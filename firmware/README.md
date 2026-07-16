# Firmware Versions

This folder keeps the main firmware stages from the ESP32 energy-meter project in a cleaner structure.

## Layout

- `01_smart_meter/`
  - Structured Modbus-style meter reader with reusable protocol/network/display helpers
- `02_energy_oled/`
  - Standalone OLED meter display
- `03_energy_mqtt/`
  - MQTT telemetry version
- `04_energy_relay/`
  - MQTT relay control version
- `05_energy_relay_sg90/`
  - Relay plus SG90 servo control
- `06_energy_relay_sg90_sensor/`
  - Relay, SG90, DHT11, and photoresistor version

All hard-coded credentials were replaced with placeholder values before publication.

