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

## Life of an Arduino Sketch

1. Creating an `.ino` file.
2. (Optionally) include custom header and source files.
3. (Optionally) include official or 3rd party libraries.
4. Build (compile) the sketch.
5. Upload (flash) the sketch.
6. The board resets and the bootloader jumps to the sketch.

## Dependencies

You will need to install

- [Arduino](https://www.arduino.cc/en/software)
- The [Espressif Systems ESP32 board manager](https://github.com/espressif/arduino-esp32)
- [WebSocket Server and Client for Arduino](https://github.com/Links2004/arduinoWebSockets)
