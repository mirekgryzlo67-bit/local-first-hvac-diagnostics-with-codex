# Local-First HVAC Diagnostics with Codex

An autonomous, cloud-independent HVAC control ecosystem for a solar thermal system and a heat recovery ventilation unit. The system combines custom ESP32 controllers, RS485, ESP-NOW, local telemetry, alarm history, and a physical service dongle.

## What the demo shows

1. A live RS485 communication fault is introduced in the solar controller network.
2. The controller detects the fault and stores machine status and alarm history.
3. A custom ESP32 service dongle transfers diagnostic data to a laptop.
4. OpenAI Codex with GPT-5.6 interprets the raw registers and explains the likely physical cause in natural language.
5. The same engineering workflow is used to develop and test a larger recuperator controller with multiple modules and more than 20 sensors.

Demo video: https://youtu.be/tw0jvQqFI-U

## Repository structure

- `solar-controller/CYD_SOLAR_OTA.ino` - touchscreen solar thermal controller with local automation, RS485, ESP-NOW, logging, alarms, and service commands.
- `service-dongle/DongleESP_UPLOAD.ino` - ESP32 service dongle used to bridge local diagnostic data to a laptop.
- `recuperator-tests/` - hardware test sketches for the recuperator dimmer, pressure sensor, and I2C multiplexer.

## How Codex and GPT-5.6 were used

The physical architecture, control concept, safety requirements, and test objectives were defined by the project author. Codex with GPT-5.6 was used as an active engineering collaborator to:

- translate control and diagnostic requirements into ESP32/Arduino C++;
- implement and debug RS485 and ESP-NOW communication;
- analyze serial logs, raw machine registers, and intermittent communication faults;
- create service-dongle commands and alarm-history reporting;
- design repeatable tests for sensors, dimmers, and actuators;
- explain faults in plain language while keeping all essential HVAC control local and deterministic.

The AI is not in the real-time safety loop. Controllers continue to operate offline; Codex is used for development and service diagnostics.

## Hardware and software

- ESP32 microcontrollers
- Arduino C++
- RS485
- ESP-NOW
- local SD logging
- touchscreen controller
- HVAC sensors and actuators
- OpenAI Codex with GPT-5.6

## Building

These sketches target specific prototype hardware and require the matching ESP32 board definitions and Arduino libraries. Before compiling:

1. Replace `YOUR_WIFI_SSID` and `YOUR_WIFI_PASSWORD` where network access is needed.
2. Set the ESP-NOW peer MAC addresses for your devices.
3. Select the correct ESP32 board and pin configuration.
4. Install the libraries referenced by each sketch.

Network credentials, device MAC addresses, and local filesystem paths have been removed from this public copy.

## Project status

This repository contains the working Build Week prototype and representative hardware test sketches. It is not a general-purpose HVAC product and should be adapted and validated for each installation.
