#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <QMC5883LCompass.h>
#include <math.h>


class SensorFusion {
    // instance vars
    QMC5883LCompass compass;
    float compassWeight;
    float ema;
    float prevYK = 0;

 public:
  SensorFusion(float compassWeight, float ema) : compassWeight(compassWeight), ema(ema) {}

  void setup() {
    compass.init();
    compass.setCalibrationOffsets(-155.00, -179.00, -1178.00);
    compass.setCalibrationScales(1.32, 1.12, 0.74);

  }

  float loopStep(float thetaK) {
    compass.read();

    float YK = ema * compass.getAzimuth() + (1 - ema) * prevYK;

    return compassWeight * thetaK + (1 - compassWeight) * YK;
  }
};

#endif