#include "display.h"
#include "../../include/wscommunicator.h"

// Global State

// Network configuration
const char* SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;
WsCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);


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
    char port[7];
    snprintf(port, 6, ":%d", wsCommunicator.getPort());
    
    display.loopStep(0, 0, wsCommunicator.getIpAddress().c_str());
    display.loopStep(0, 1, port);
}
