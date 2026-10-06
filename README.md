# Automotive ADAS Sensor Fusion & CAN ECU Firmware

[![Language: Embedded C](https://img.shields.io/badge/Language-Embedded%20C%20%2F%20C%2B%2B-blue.svg)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Target: 32-bit Microcontroller](https://img.shields.io/badge/Target-ARM%20Cortex--M%20%2F%2032--bit%20MCU-orange.svg)](https://www.arm.com/)
[![Protocol: CAN 2.0B](https://img.shields.io/badge/Protocol-CAN%20Bus%202.0B-green.svg)](https://en.wikipedia.org/wiki/CAN_bus)
[![Tests: 100% Passing](https://img.shields.io/badge/Tests-100%25%20Passing-brightgreen.svg)](https://github.com/arigisurendranaidu2005-code/automotive-adas-can-ecu-firmware)
[![Standard: AS&UX / MISRA-C](https://img.shields.io/badge/Standard-AS%26UX%20%2F%20MISRA--C-red.svg)](https://www.misra.org.uk/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A production-grade automotive embedded software module engineered for a 32-bit Electronic Control Unit (ECU). Implements real-time proximity radar/ultrasonic sensor fusion, Controller Area Network (CAN 2.0B) frame broadcasting with CRC-16 data integrity, a deterministic finite state machine for Emergency Brake Assist, and a hardware watchdog monitor aligned with Aptiv's **Advanced Safety & User Experience (AS&UX)** standards.

---

## 🏛️ System Architecture & AS&UX Design

```
   ┌──────────────────────────────────────────────┐
   │         Proximity Sensors (Radar/Sonar)      │
   └──────────────────────┬───────────────────────┘
                          │ Raw Telemetry (10ms Periodic)
   ┌──────────────────────▼───────────────────────┐
   │          Sensor Fusion & Filtering           │
   │  • Rolling Window Moving Average Filter      │
   │  • Relative Velocity & Time-to-Collision     │
   │  • Multi-Tier Hazard Classification         │
   └──────────────────────┬───────────────────────┘
                          │ Hazard Assessment
   ┌──────────────────────▼───────────────────────┐
   │        Deterministic ECU State Machine       │
   │  • INIT ➔ NORMAL ➔ WARNING ➔ BRAKE ➔ FAULT   │
   │  • Independent Hardware Watchdog Monitor    │
   └──────────────────────┬───────────────────────┘
                          │ CAN Message Dispatch (50ms)
   ┌──────────────────────▼───────────────────────┐
   │             CAN 2.0B Bus Driver              │
   │  • Frame Packing (ID: 0x120, DLC: 8 Bytes)  │
   │  • CRC-16-CCITT Error Detection Guardrail    │
   └──────────────────────────────────────────────┘
```

---

## 💻 Core Capabilities & Engineering Practices

1. **Embedded C Implementation:** Structured according to MISRA-C principles (bounded loops, no dynamic memory allocation, strict pointer validity checks).
2. **Sensor Noise Filtering:** Rolling buffer filter preventing false positive collision triggers caused by transient ultrasonic reflections.
3. **Automotive CAN 2.0B Bus Frame:** Standardized cyclic telemetry broadcast (CAN ID `0x120`) carrying filtered obstacle distance, active state, and hazard levels with 16-bit CRC payload verification.
4. **Safety & Fault Mitigation:** Watchdog supervisor transitioning the vehicle into a fail-safe state (`ECU_STATE_FAULT`) in case of missed scheduling deadlines.

---

## 🧪 Testing & Verification (100% Pass Rate)

Automated unit tests execute deterministically against the simulated ECU hardware abstraction layer:

```bash
# Compile and run unit tests
gcc -Wall -Wextra -Iinclude src/can_protocol.c src/sensor_fusion.c src/safety_ecu.c tests/test_ecu.c -o tests/test_ecu.exe
.\tests\test_ecu.exe
```

### Verified Test Cases:
- ✅ `test_can_crc_and_packing`: Validates bitwise packing, unpack consistency, and frame corruption detection.
- ✅ `test_sensor_fusion_filtering`: Verifies multi-sample smoothing and velocity derivation.
- ✅ `test_emergency_braking_state_machine`: Verifies emergency brake actuation threshold ($\le 50\text{ cm}$).
- ✅ `test_watchdog_failure_detection`: Validates watchdog expiration handling and transition into fail-safe mode.

---

## 🚀 Quickstart & Build Instructions

```bash
# Clone the repository
git clone https://github.com/arigisurendranaidu2005-code/automotive-adas-can-ecu-firmware.git
cd automotive-adas-can-ecu-firmware

# Compile and run demo simulation
make
.\adas_ecu_demo.exe
```

---

## 📜 License
MIT License. Authored by Surendra Naidu Arigi. Aligned with Aptiv AS&UX Automotive Standards.
