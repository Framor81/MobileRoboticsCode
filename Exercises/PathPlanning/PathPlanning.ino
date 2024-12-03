#include <WString.h>    

#include "../../include/wscommunicator.h"
#include "../../include/motorcontrol.h"
#include "../display/display.h"
#include "../../include/forwardkinematics.h"
#include "../PositionControl/positioncontrol.h"

#include <math.h>

// Physical characteristics
const float WHEEL_DIAMETER = 0.086;
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI;
const float TRACK_WIDTH = 0.16;


// Network Configuration
const char* SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;
char message[100];

WsCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);

const unsigned long MESSAGE_INTERVAL = 500;
IntervalTimer messageTimer(500);

// Display Configuration
Display display;

// Control characteristics

const float LEFT_GAIN = 0.5;
const float RIGHT_GAIN = 0.5;
const float MAX_VELOCITY_STEP = 0.5;
const float MAX_LINEAR_VELOCITY = 0.4;
const float MIN_PWM_PERCENT = 30;
const unsigned long MOTOR_CONTROL_INTERVAL = 100;


MotorControl motorControl(
    WHEEL_CIRCUMFERENCE, LEFT_GAIN, RIGHT_GAIN, MAX_VELOCITY_STEP, MAX_LINEAR_VELOCITY, MIN_PWM_PERCENT, MOTOR_CONTROL_INTERVAL
);

// Kinematics configuration

const unsigned long FORWARD_KINEMATICS_INTERVAL = 250;

ForwardKinematics forwardKinematics(TRACK_WIDTH, FORWARD_KINEMATICS_INTERVAL);

// Position control configuration
int path[28][2] = {
    {1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}, {1, 8}, {1, 9}, {1, 10}, 
    {1, 11}, {1, 12}, {1, 13}, {1, 14}, {1, 15}, {1, 16}, {1, 17}, {1, 18}, {1, 19}, {1, 20}, 
    {1, 21}, {1, 22}, {1, 23}, {1, 24}, {1, 25}, {1, 26}, {0, 26}
};
int pathcount = 1;
const float GOAL_THRESHOLD = 0.1;

const float MAX_ANGULAR_VELOCITY = 1.0;

// modify these to our particular robot
const float K_POSITION = 1.0; 
const float K_ORIENTATION = 2.0;

const unsigned long POSITION_CONTROL_INTERVAL = 250;


PositionControl positionControl(path[0][0], path[0][1], GOAL_THRESHOLD, MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY, K_POSITION, K_ORIENTATION, TRACK_WIDTH, POSITION_CONTROL_INTERVAL);


void setup() { 
    Serial.begin(115200);

    wsCommunicator.setup(); 
    display.setup();

    char port[7];
    snprintf(port, 6, ":%d", wsCommunicator.getPort());

    display.drawString(0, 0, "Path Planning");
    display.drawString(0, 1, wsCommunicator.getIpAddress().c_str());
    display.drawString(0, 2, port);

    motorControl.setup();
    
    // update for different initial paths
    forwardKinematics.setup(0, 0, 0);

    positionControl.setup();
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

    // Update position control
    Pose pose = forwardKinematics.getPose();
    bool shouldUpdateVelocities = positionControl.loopStep(pose, leftVelocity, rightVelocity);

    if (shouldUpdateVelocities){
        motorControl.setTargetVelocity(leftVelocity, rightVelocity);
    }

    if (positionControl.atGoal and pathcount < (sizeof(path) / sizeof(path[0]))) {
        positionControl.updateGoal(path[pathcount][0], path[pathcount][1]);
        pathcount++;
    }

    // Send message over WebSocket
    if (messageTimer) {
        snprintf(
            message, sizeof(message), "x=%f y=%f theta=%f vl=%f vr=%f", pose.x, pose.y, pose.theta, leftVelocity, rightVelocity
        );
        wsCommunicator.sendText(message, strlen(message));
    }
}
