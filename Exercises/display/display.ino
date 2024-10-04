#include "display.h"

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
    auto wrappedCB = [this](uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
      wsEventCB(*this, num, type, payload, length);
    };
    webSocket.begin();
    webSocket.onEvent(wrappedCB);
    // Serial.printf("[COMMUNICATOR::SETUP] WebSocket server at ws://%d.%d.%d.%d:%d\n", ip[0], ip[1], ip[2], ip[3], port);

    display.loopStep("%d.%d.%d.%d:%d\n", ip[0], ip[1], ip[2], ip[3], PORT);
}
