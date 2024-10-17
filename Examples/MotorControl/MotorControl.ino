#include "../../include/motorcontrol.h"
#include "../../include/wscommunicator.h"

//
// Global state
//

// Network configuration
const char* SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;
WsCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);

// Motor configuration
DualMotorDriver motorDriver;

//
// Setup
//

void setup() {
  Serial.begin(115200);
  wsCommunicator.setup();
  motorDriver.setDirection(DIRECTION_FORWARD);
  motorDriver.setPwmPercent(50);
}

//
// Loop
//

void loop() {
  wsCommunicator.loopStep();

  if (wsCommunicator.isEnabled()) {
    motorDriver.setPwmPercent(50);
  } else {
    motorDriver.stop();
  }
}
