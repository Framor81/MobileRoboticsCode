#include "../../include/wscommunicator.h"
#include "../../include/motorcontrol.h"
#include "../display/display.h"
#include "../Kinematics/kinematics.h"
#include <iostream>
#include <math.h>



// Initialize a WsCommunicator object
const char* SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;
WsCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);

// Initialize a MotorControl object
MotorControl motorControl(0.086 * PI, 0.1, 0.1, 0.05, 0.5, 0, 250);

// Initialize a Display object
Display display;

// Initialize a Kinematics object
Kinematics kinematics(0, 0, 0, 250);
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
    motorControl.setup();
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

    char port[7];
    snprintf(port, 6, ":%d", wsCommunicator.getPort());
    display.loopStep(0, 0, wsCommunicator.getIpAddress().c_str());
    display.loopStep(0, 1, port);

    if (timeRan - startTime >= 10000){
        motorControl.stop();
    } else {
        motorControl.loopStep(true);
        kinematics.loopStep(motorControl.getLeftVelocity(), motorControl.getRightVelocity());
        // Serial.println("xG = " + kinematics.xG + "\nyG = " + kinematics.yG + "\nThetaG =  " + kinematics.thetaG );   
        Serial.printf("%f, %f, %f\n", kinematics.xG, kinematics.yG, kinematics.thetaG); 
    }
}