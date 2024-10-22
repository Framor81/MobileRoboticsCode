#include <cmath>
#include "../../include/intervaltimer.h"

#ifndef KINEMATICS_H
#define KINEMATICS_H

class Kinematics {
    public:
        float xG, yG, thetaG, xDotR, yDotR, thetaDotR;
        const float R = 4.3, D = 16;
        IntervalTimer timer;

        Kinematics(float xG, float yG, float thetaG, unsigned long interval) : timer(interval) {
            this->xG = xG;
            this->yG = yG;
            this->thetaG = thetaG;
        }

        void setup() {
        }

        void loopStep(float phiDotL, float phiDotR) {
            // update local reference frame
            float deltaT = timer.getLastDelta() / 1000;

            xDotR = (phiDotL + phiDotR) * 0.5;
            yDotR = 0;
            thetaDotR = ((phiDotR * R) - (phiDotL * R)) / D;
            
            xG += (xDotR * cos(thetaG)- yDotR * sin(thetaG)) * deltaT;
            yG += (xDotR * sin(thetaG) + yDotR * cos(thetaG)) * deltaT;
        
            thetaG += thetaDotR * deltaT;
        }
};

#endif