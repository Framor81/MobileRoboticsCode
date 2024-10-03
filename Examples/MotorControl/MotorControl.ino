#include "../../include/motorcontrol.h"
#include "../../include/wscommunicator.h"

//
// Global state
//

// Network configuration
const char* SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;
WSCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);

// Motor configuration
MotorControl motorControl;

//
// Setup
//

void setup() {
  Serial.begin(115200);
  wsCommunicator.setup();
  motorControl.setup();
  motorControl.setDirection(DIRECTION_FORWARD);
  motorControl.setPWMPercent(50);
}

//
// Loop
//

void loop() {
  wsCommunicator.loopStep();

  if (wsCommunicator.isEnabled()) {
    motorControl.setPWMPercent(50);
  }

  motorControl.loopStep(wsCommunicator.isEnabled());
}
