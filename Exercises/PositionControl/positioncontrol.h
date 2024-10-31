#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include "../../include/forwardkinematics.h"
#include "../../include/intervaltimer.h"
#include <math.h>

class PositionControl {
    float goalX;
    float goalY;
    float goalThreshold;

    float maxLinearVelocity;
    float maxAngularVelocity;

    float K_position;
    float K_orientation;

    float trackWidth;

    IntervalTimer updateTimer;

    public: 
    PositionControl(
        float goalX, float goalY, float goalThreshold, float maxLinearVelocity, 
        float maxAngularVelocity, float K_position, float trackWidth, float K_orientation, unsigned long interval
    )
        : goalX(goalX)
        , goalY(goalY)
        , goalThreshold(goalThreshold)
        , maxLinearVelocity(maxLinearVelocity)  
        , maxAngularVelocity(maxAngularVelocity)
        , K_position(K_position)
        , K_orientation(K_orientation)
        , trackWidth(trackWidth)
        , updateTimer(interval) {}

    void setup() {}

    bool loopStep(Pose pose, float& leftVelocity, float& rightVelocity) {
        if (!updateTimer){
            return false;
        }

        float d = sqrt((goalX - pose.x) * (goalX - pose.x) + (goalY - pose.y) * (goalY - pose.y));
        
        if (d < goalThreshold){
            leftVelocity = 0;
            rightVelocity = 0;  
            return true;
        }

        float angleToGoal = atan2(goalY - pose.y, goalX - pose.x);

        float angleError = angleToGoal - pose.theta;
        
        float v = min(K_position * d, maxLinearVelocity);
        float thetaDot = min(K_orientation * angleError, maxAngularVelocity);

        leftVelocity = v - thetaDot / trackWidth / 2.0;
        rightVelocity = v + thetaDot * trackWidth / 2.0;

        return true;
    }

};

#endif