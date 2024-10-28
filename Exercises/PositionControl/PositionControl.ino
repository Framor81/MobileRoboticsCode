#include <WString.h>    

#include "../../include/wscommunicator.h"
#include "../../include/motorcontrol.h"
#include "../display/display.h"
#include "../../include/forwardkinematics.h"

// Network Configuration
const char* SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;
char message[100];

WsCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);

const unsigned long MESSAGE_INTERVAL = 500;
IntervalTimer messageTimer(500);

// Physical characteristics
const float WHEEL_DIAMETER = 0.086;
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI;
const float TRACK_WIDTH = 0.16;

// Display Configuration
Display display;

// Control characteristics

const float LEFT_GAIN = 0.5;
const float RIGHT_GAIN = 0.5;
const float MAX_VELOCITY_STEP = 0.5;
const float MAX_LINEAR_VELOCITY = 0.4;
const float MIN_PWM_PERCENT = 0.15;
const unsigned long MOTOR_CONTROL_INTERVAL = 100;


MotorControl motorControl(
    WHEEL_CIRCUMFERENCE, LEFT_GAIN, RIGHT_GAIN, MAX_VELOCITY_STEP, MAX_LINEAR_VELOCITY, MIN_PWM_PERCENT, MOTOR_CONTROL_INTERVAL
);

// Kinematics configuration

const unsigned long FORWARD_KINEMATICS_INTERVAL = 250;

ForwardKinematics forwardKinematics(TRACK_WIDTH, FORWARD_KINEMATICS_INTERVAL);

void setup() { 
    wsCommunicator.setup(); 
    display.setup();

    char port[7];
    snprintf(port, 6, ":%d", wsCommunicator.getPort());

    display.drawString(0, 0, "Position Control");
    display.drawString(0, 1, wsCommunicator.getIpAddress().c_str());
    display.drawString(0, 2, port);

    motorControl.setup();
    motorControl.setTargetVelocity(0.5);

    forwardKinematics.setup();
}

// Reset
void reset() {
    forwardKinematics.setPose(0, 0, 0);
}

// Loop
void loop() {
    // Update websocket interface
    wsCommunicator.loopStep();

    if (wsCommunicator.resetFlagIsSet()) {
        reset();
        wsCommunicator.clearResetFlag();
    }

    // Update the OLED display
    display.loopStep();

    // Update the motors (they are halted if we don't receive heartbeats)
    motorControl.loopStep(wsCommunicator.isEnabled());

    // Update forward kinematics
    float leftVelocity = motorControl.getLeftVelocity();
    float rightVelocity = motorControl.getRightVelocity();
    forwardKinematics.loopStep(leftVelocity, rightVelocity);
    
    if (messageTimer) {
        Pose pose = forwardKinematics.getPose();

        snprintf(
            message, sizeof(message), "x = %f, y = %f, theta = %f, vl = %f, vr = %f", pose.x, pose.y, pose.theta, leftVelocity, rightVelocity
        );
        wsCommunicator.sendText(message, strlen(message));
    }
}
