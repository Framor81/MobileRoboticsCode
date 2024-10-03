#ifndef INTERVAL_TIMER_H
#define INTERVAL_TIMER_H

class IntervalTimer {
 private:
  unsigned long interval;
  unsigned long lastTime;

 public:
  IntervalTimer(unsigned long interval) : interval(interval), lastTime(0) {}

  void setLastTime(unsigned long lastTime) { this->lastTime = lastTime; }

  // The boolean operator contains the most pertinent logic for the IntervalTimer class.
  operator bool() {
    unsigned long now = millis();
    if (now - lastTime >= interval) {
      lastTime = now;
      return true;
    }
    return false;
  }
};

#endif
