#include "../../include/intervaltimer.h"
#include "../../include/motorcontrol.h"

Encoder left(LEFT_ENCODER_DATA_PIN, LEFT_ENCODER_CLOCK_PIN);
Encoder right(RIGHT_ENCODER_DATA_PIN, RIGHT_ENCODER_CLOCK_PIN);

IntervalTimer printTimer(500);

void setup() {
  Serial.begin(115200);
  left.setup();
  right.setup();
}

void loop() {
  if (printTimer) {
    Serial.print("Left: ");
    left.getRotationsPerSecond();
    Serial.print("Right: ");
    right.getRotationsPerSecond();
  }
}
