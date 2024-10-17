#include <cmath>

class Kinematics {
    // initialize xG, yG, thetaG as the x-coordinate, y-coordinate, and angle in the global frame
    // initialize x dot R, y dot R, theta dot R as the x-coordinate, y-coordinate, and angle in the local reference frame
    // makes sense for us to use fractional values
    float xG, yG, thetaG, xDotR, yDotR, thetaDotR;

    // initialize r and d as the wheel radius and distance between the two motors
    const float R = 6.2, D = 16; // units in cm

    void update(float phiDotL, float phiDotR) {
        // update local reference frame
        xDotR = (phiDotL + phiDotR) * 0.5;
        yDotR = 0;
        thetaDotR = ((phiDotR * R) - (phiDotL * R)) / D;
        
        // update global reference
        xG += xDotR * cos(thetaG)- yDotR * sin(thetaG);
        yG += xDotR * sin(thetaG) + yDotR * cos(thetaG);
        thetaG += thetaDotR;
    }    

};

int main() {
    return 0;
}