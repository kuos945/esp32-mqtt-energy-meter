# esp32-mqtt-energy-meter

ESP32 energy meter project collection for reading a smart meter, showing values on an OLED, publishing telemetry to MQTT, and extending the system with relay and SG90 control.

This repository is intended to package the main results of the current workspace into one GitHub-ready project. The code was developed in several stages, from a simple OLED meter display to MQTT reporting, relay control, and sensor-driven automation.

## Project Goals

- Read real-time electrical measurements from a meter over `Serial2`
- Display the readings on an OLED using either `Adafruit_SSD1306` or `U8g2`
- Publish meter data to an MQTT broker as JSON
- Receive MQTT commands to control a relay
- Extend the system with additional sensors, a photoresistor, DHT11, and an SG90 servo

## Repository Layout

- `firmware/`
  - Cleaned source tree with the main firmware versions
  - Includes the reusable `meter_protocol.h`, `meter_network.h`, and `meter_display.h` helpers
- `secrets.example.h`
  - Placeholder template for Wi-Fi and MQTT credentials
- `.gitignore`
  - Excludes build output and local secret files

For a version-by-version breakdown, see [firmware/README.md](/D:/0709-ESP32/esp32-mqtt-energy-meter/firmware/README.md).

## Main Firmware Architecture

Across the newer sketches, the firmware follows the same overall flow:

1. Initialize serial, OLED, Wi-Fi, and MQTT.
2. Query the meter frame from `Serial2`.
3. Decode raw register values into engineering units.
4. Render the current state to the display.
5. Publish the measurement JSON to MQTT.
6. Optionally receive MQTT commands and drive outputs such as relay or servo.

## Data Model

The meter reading is represented by a small structure containing:

- `voltage`
- `current`
- `power`
- `energy`
- `frequency`
- `pf` or `powerFactor`
- `valid`

In the structured version, the meter protocol helper also tracks:

- slave ID
- function code
- exception code
- raw register values
- frame timestamp

## Meter Communication

The project uses a serial request/response pattern to query the energy meter.

- Request is sent through `Serial2`
- The response frame is read with a short timeout
- The code checks the expected header and frame length
- Raw values are scaled into human-readable units

In the more general `14_smart_meter` version, the protocol is wrapped in reusable helpers:

- `crc16Modbus(...)`
- `buildReadHoldingRequest(...)`
- `readExact(...)`
- `parseReadHoldingResponse(...)`
- `requestAndRead(...)`

## Display Logic

The display code changed over time from a basic OLED text layout to a cleaner modular renderer.

### OLED-only versions

- Show live energy readings in a compact text layout
- Include a boot screen and status messages

### U8g2 structured version

- Splits rendering into reusable functions
- Displays meter, Wi-Fi, and MQTT status lines separately
- Uses a consistent 128x64 layout

### Sensor-enhanced version

- Alternates between a meter page and a sensor page
- Shows temperature, humidity, and light level
- Keeps relay and servo state visible for debugging

## MQTT Design

The MQTT versions publish the meter readings as JSON.

Typical payload shape:

```json
{
  "V": 110.2,
  "A": 0.532,
  "W": 58.4,
  "kWh": 1.274,
  "F": 60.0,
  "PF": 0.97
}
```

Later versions add:

- device ID
- validity flag
- timestamp in milliseconds
- relay command subscription topic

## Hardware Notes

Common pin mapping used in the newer sketches:

- OLED SDA: `21`
- OLED SCL: `22`
- Meter RX: `16`
- Meter TX: `17`
- Relay: `18`
- SG90 servo: `5`
- Photoresistor: `33`
- DHT11: `14`

The exact wiring depends on your board and meter module, but these are the default pins used in the code now present in the workspace.

## Libraries

The project uses the following libraries:

- `WiFi`
- `PubSubClient`
- `ArduinoJson`
- `Adafruit_GFX`
- `Adafruit_SSD1306`
- `U8g2`
- `SimpleDHT`

## Security Note

Do not commit real Wi-Fi or MQTT credentials to GitHub.

Recommended approach:

- put secrets in a local `secrets.h`
- keep `secrets.h` out of git
- commit a `secrets.example.h` template instead

## Suggested GitHub Cleanup

Before publishing:

- remove `build/` folders and other generated outputs
- keep only source sketches and headers
- use the `firmware/` tree as the published source layout
- keep placeholder credentials or move secrets into a local `secrets.h`

## What This Repo Represents

This repository is best understood as a project diary plus a deployable firmware collection:

- early meter-display experiments
- MQTT telemetry integration
- relay control via MQTT
- sensor-driven expansion with SG90 and DHT11
- a reusable smart-meter protocol layer

If you want, the next step is to split these sketches into a cleaner GitHub structure and add a `secrets.example.h` plus an `.ino` entry point for the final firmware version.
