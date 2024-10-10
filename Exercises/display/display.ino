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

    // u8x8.setFont(u8x8_font_chroma48medium8_r);
    // u8x8.drawString(0, 0, wsCommunicator.getIpAddress().c_str());

    char port[7];
    snprintf(port, 6, ":%d", wsCommunicator.getPort());
    // u8x8.drawString(0, 1, port);
    
    display.loopstep(0, 0, wsCommunicator.getIpAddress().c_str());
    display.loopstep(0, 1, port);
}
