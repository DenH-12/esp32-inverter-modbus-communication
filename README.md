# ESP32 Solar Inverter Modbus Communication

ESP32-based firmware for monitoring and controlling hybrid solar inverters via Modbus RTU (RS485/UART). Includes a web interface, Home Assistant integration, and OTA updates.

## Key Features

- **Web Interface**: Real-time status monitoring, system diagnostics, and configuration management.
- **Home Assistant Integration**: MQTT Auto-Discovery support.
- **OTA Updates**: Over-the-air firmware and SPIFFS updates.

---

## Flash / Installation

### Option 1: Web Installer (Browser)
Flashing via **ESP Web Tools** (Chrome / Edge):
1. Open the [Web Installer Page](https://denh-12.github.io/esp32-inverter-modbus-communication/).
2. Connect your ESP32 via USB and click **Install**.

### Option 2: Flash `full-firmware.bin` via `esptool`
Download `full-firmware.bin` from the latest [Release](https://github.com/DenH-12/esp32-inverter-modbus-communication/releases) and flash it at offset `0x0`:

```bash
esptool --chip esp32 --port /dev/ttyUSB0 --baud 460800 write-flash 0x0 full-firmware.bin
```
(On Windows, replace /dev/ttyUSB0 with your COM port, e.g., COM3)

## Accessing the Web UI

After flashing the firmware:
1. Connect to the ESP32 Wi-Fi Access Point (AP) named **ESP32_Solar_********.
2. Open your web browser and navigate to: [http://192.168.4.1](http://192.168.4.1)
<img width="1136" height="839" alt="image" src="https://github.com/user-attachments/assets/bf32271d-7cb9-4b2f-8b0b-dad5dee4acc9" />
