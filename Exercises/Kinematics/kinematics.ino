#include "kinematics.h"

// Global State


// Initialize display
Kinematics kinematics;

//
// Setup
//

void setup() {
    kinematics.setup(); 
}

//
// Loop
//

void loop() {
    // access encoder to get phiDotL and phiDotR values
    // phiDotL = leftEncoderAccess
    // phiDotR = rightEncoderAccess
    kinematics.loopstep(phiDotL, phiDotR);  
}
