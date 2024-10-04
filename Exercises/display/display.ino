#include "display.h"
#include "../../include/wscommunicator.h"

// Global State

// Network configuration
const char* SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;
WSCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);

// Initialize display
Display display;

//
// Setup
//

void setup() {
    Serial.begin(115200);
    wsCommunicator.setup();
    display.setup();
}

//
// Loop
//

void loop() {
    // Serial.printf("[COMMUNICATOR::SETUP] WebSocket server at ws://%d.%d.%d.%d:%d\n", ip[0], ip[1], ip[2], ip[3], port);
    
    display.loopStep(wsCommunicator.getIpAddress() + wsCommunicator.getPort());
}
