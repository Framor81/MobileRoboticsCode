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

    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.drawString(0, 0, wsCommunicator.getIpAddress().c_str());

    char port[7];
    snprintf(port, 6, ":%d", wsCommunicator.getPort());
    u8x8.drawString(0, 1, port);

  
}

//
// Loop
//

void loop() {
    // Serial.printf("[COMMUNICATOR::SETUP] WebSocket server at ws://%d.%d.%d.%d:%d\n", ip[0], ip[1], ip[2], ip[3], port);
    // char buffer[22];
    // snprintf(buffer, 22, "%s:%d", wsCommunicator.getIpAddress(), wsCommunicator.getPort());
    // display.loopStep(buffer);
    char port[7];
    snprintf(port, 6, ":%d", wsCommunicator.getPort());
    

    // display.loopStep(wsCommunicator.getIpAddress(), 0, 16);
    // display.loopStep(port, 0, 32);
}
