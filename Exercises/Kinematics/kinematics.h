#include <cmath>

#ifndef KINEMATICS_H
#define KINEMATICS_H

class Kinematics {
    public:
        float xG, yG, thetaG, xDotR, yDotR, thetaDotR;
        const float R = 6.2, D = 16;

        Kinematics(float xG, float yG, float thetaG) {
            this->xG = xG;
            this->yG = yG;
            this->thetaG = thetaG;
        }

        void setup() {
        }

        void loopStep(float phiDotL, float phiDotR) {
            // update local reference frame
            xDotR = (phiDotL + phiDotR) * 0.5;
            yDotR = 0;
            thetaDotR = ((phiDotR * R) - (phiDotL * R)) / D;
                
            
            xG += xDotR * cos(thetaG)- yDotR * sin(thetaG);
            yG += xDotR * sin(thetaG) + yDotR * cos(thetaG);
        
            thetaG += thetaDotR;
        }
};

#endif