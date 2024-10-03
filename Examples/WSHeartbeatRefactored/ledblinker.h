typedef enum { LED_ON = LOW, LED_OFF = HIGH } LedState;

class LedBlinker {
 public:
  LedState ledState;
  unsigned long blinkInterval;
  unsigned long blinkLastTime;

  LedBlinker(unsigned long interval) : ledState(LED_OFF), blinkInterval(interval), blinkLastTime(0) {}

  void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, ledState);
  }

  void loopStep(bool isEnabled) {
    // Update for blinking the builtin LED
    if (isEnabled) {
      unsigned long now = millis();
      if (now - blinkLastTime > blinkInterval) {
        blinkLastTime = now;

        ledState = (ledState == LED_ON) ? LED_OFF : LED_ON;
        digitalWrite(LED_BUILTIN, ledState);
      }
    } else {
      digitalWrite(LED_BUILTIN, LED_OFF);
    }
  }
};
