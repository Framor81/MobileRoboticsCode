#include "../../include/wscommunicator.h"
#include "../../include/motorcontrol.h"
#include "../display/display.h"
#include "../Kinematics/kinematics.h"
#include <iostream>



// Initialize a WsCommunicator object
const char* SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;
WsCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);

// Initialize a MotorControl object
MotorControl motorControl;

// Initialize a Display object
Display display;

// Initialize a Kinematics object
Kinematics kinematics;
unsigned long startTime = 0;
unsigned long timeRan = 0;

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
    motorControl.setup(6.2, 0.1, 0.1, 0.05, 0.5, 0, 250);
    motorControl.setTargetVelocity(0.2, 0.25);  
    startTime = millis();
}


// Loop:
//     Update the wsCommunicator
//     Update the motorControl
//     Update the kinematics
//     Output the current pose
void loop(){
    timeRan = millis();

    wsCommunicator.loopStep();
    display.loopStep();

    if (timeRan - startTime >= 10000){
        motorControl.stop();
    } else {
        motorControl.loopStep(wsCommunicator.isEnabled());
        kinematics.loopStep(motorControl.getLeftVelocity(), motorControl.getRightVelocity());
        Serial.println("xG = " + kinematics.xG + "\nyG = " + kinematics.yG + "\nThetaG =  " + kinematics.thetaG );    
    }
}