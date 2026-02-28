# Robot Wireless Communications

Firmware framework for wireless communication between a centralized ESP-based transmitter (base station) and multiple heterogeneous robot platforms using different microcontrollers and communication protocols.

This project demonstrates scalable, modular communication design for distributed robotic systems.

---

## Overview

This repository implements a wireless communication architecture where:

* An ESP32-based base station acts as a central transmitter
* Multiple robots act as receivers
* Each robot may use a different microcontroller
* Different communication protocols are supported (Wi-Fi, ESP-NOW, BLE, Bluetooth Classic, etc.)

The system enables command broadcasting, robot addressing, and telemetry exchange across heterogeneous embedded platforms.

---

## System Architecture

Base Station (ESP32)

* Central controller
* Sends structured packets
* Handles device addressing
* Supports multiple wireless protocols

Robot Nodes

* Built on different microcontrollers
* Parse incoming packets
* Execute commands
* Optionally send telemetry back

The architecture is modular, allowing additional robots and protocols to be integrated with minimal changes.

---

## Supported Communication Protocols

Depending on implementation folders in the repository:

* ESP-NOW (low-latency peer-to-peer)
* Wi-Fi (TCP/UDP)
* BLE
* Bluetooth Classic
* UART (for wired testing/debugging)

Each protocol implementation follows a structured packet format to maintain interoperability.

---

## Packet Structure

All communications follow a structured format:

* Start byte / header
* Robot ID
* Command type
* Payload data
* Checksum / validation (if implemented)

This ensures:

* Reliable parsing
* Multi-robot addressing
* Expandability for new commands

---

## Features

* Multi-robot communication from a single base station
* Heterogeneous microcontroller compatibility
* Protocol abstraction
* Modular firmware structure
* Command-based architecture
* Scalable addressing strategy

---

## Use Cases

* Multi-robot coordination experiments
* Swarm robotics prototyping
* Distributed robotic control systems
* Embedded systems communication research
* Robotics lab demonstrations

---

<!-- ## Repository Structure

Example structure (adapt as needed):

```
robot_wireless_communications/
│
├── base_station/
│   ├── esp_now/
│   ├── wifi/
│   └── bluetooth/
│
├── robot_receivers/
│   ├── robot_type_1/
│   ├── robot_type_2/
│   └── robot_type_3/
│
└── common/
    ├── packet_format.h
    └── utilities/
``` -->

Each robot folder contains firmware specific to its microcontroller and protocol.

---

## Hardware Requirements

Base Station:

* ESP32 development board

Robot Nodes:

* Any supported microcontroller (ESP32, Arduino, STM32, etc.)
* Wireless module (if not integrated)

---

## Getting Started

1. Flash base station firmware to ESP32.
2. Flash corresponding receiver firmware to robot microcontroller.
3. Configure MAC addresses / IP addresses (if required).
4. Power devices.
5. Monitor communication via serial output.

---

## Design Principles

* Hardware abstraction where possible
* Structured and predictable packet design
* Clear separation between communication and control logic
* Scalability for future swarm or fleet systems
* Protocol flexibility without rewriting core logic

---

## Future Improvements

* Bi-directional telemetry standardization
* ROS2 bridge integration
* Encrto represent professionally.ypted communication
* Time synchronization across robots
* Dynamic robot discovery
* Swarm coordination layer

---

## Author

Rachna Kadam
MS Robotics | Embedded Systems | Distributed Robotic Architectures
