#include "ledblinker.h"
#include "wscommunicator.h"

//
// Global state
//

// Network configuration
const char* SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;
WSCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);

// LED Blinking configuration
const unsigned long BLINK_INTERVAL = 250;
LedBlinker ledBlinker(BLINK_INTERVAL);

//
// Setup
//

void setup() {
  Serial.begin(115200);
  wsCommunicator.setup();
  ledBlinker.setup();
}

//
// Loop
//

void loop() {
  wsCommunicator.loopStep();
  ledBlinker.loopStep(wsCommunicator.isEnabled());
}
