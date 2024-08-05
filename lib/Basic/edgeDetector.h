#ifndef __EDGEDETECTOR
#define __EDGEDETECTOR

#include <Arduino.h>

class EdgeDetector {
  public:
    EdgeDetector(uint32_t debounceTime = 50) 
      : debounceTime(debounceTime), lastState(false), lastDebounceTime(0), lastReading(false) {}

    void begin() {
      lastState = false;
    }

    bool detectRisingEdge(bool input) {
      bool reading = input;
      if (reading != lastReading) {
        lastDebounceTime = millis();
      }
      lastReading = reading;

      if ((millis() - lastDebounceTime) > debounceTime) {
        if (lastState == false && reading == true) {
          lastState = reading;
          return true;
        }
        lastState = reading;
      }
      return false;
    }

    bool detectFallingEdge(bool input) {
      bool reading = input;
      if (reading != lastReading) {
        lastDebounceTime = millis();
      }
      lastReading = reading;

      if ((millis() - lastDebounceTime) > debounceTime) {
        if (lastState == true && reading == false) {
          lastState = reading;
          return true;
        }
        lastState = reading;
      }
      return false;
    }

  private:
    uint32_t debounceTime;
    bool lastState;
    uint32_t lastDebounceTime;
    bool lastReading;
};

#endif
