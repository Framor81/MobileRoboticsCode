# Mobile Robotics Code

Code for CS 181R---Mobile Robotics---at Pomona College.

## Directory Structure

```text
.
├── Examples
│  ├── MotorControl
│  ├── WSHeartbeat
│  └── WSHeartbeatRefactored
├── include
│  ├── motorcontrol.h
│  └── wscommunicator.h
└── README.md
```

The Arduino-based motor code is split into the `Examples` and `include` directories. The `include` directory contains headers files for use in all complete programs.

## Dependencies

You will need to install

- [Arduino](https://www.arduino.cc/en/software)
- The [Espressif Systems ESP32 board manager](https://github.com/espressif/arduino-esp32)
- [WebSocket Server and Client for Arduino](https://github.com/Links2004/arduinoWebSockets)
