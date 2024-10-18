#include "../../include/wscommunicator.h"
#include "../../include/motorcontrol.h"
#include "../display/display.h"
#include "kinematics.h"



// Initialize a WsCommunicator object
const char* SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;
WSCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);

// Initialize a MotorControl object
MotorControl motorControl;

// Initialize a Display object
Display display;

// Initialize a Kinematics object
Kinematics kinematics;

// Setup:
//     Start serial
//     Start the wsCommunicator
//     Start the motorControl
//     Set the motor target velocity
//     Start the display
//     Display the IP address
//     Start the kinematics

void setup() {
    // start serial and wsCommunicator
    Serial.begin(115200);
    wsCommunicator.setup();

    display.setup();

    // START
    motorControl.setup(6.2, 0.1, 0.1, 0.05, 0.5, 0, 500);
    motorControl.setTargetVelocity(0.2, 0.25);    
}


// Loop:
//     Update the wsCommunicator
//     Update the motorControl
//     Update the kinematics
//     Output the current pose
void loop(){
    
    wsCommunicator.loopStep();
    display.loopStep();
    motorControl.loopStep();

    kinematics.loopStep(motorControl.getLeftVelocity(), motorControl.getRightVelocity());

    

}