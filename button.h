#ifndef BUTTON_H
#define BUTTON_H

#define DEBOUNCE_INTERVAL_MS (40)             // Unstable/debounce duration

struct button {
  unsigned long last_change_ms;
  uint8_t pin;
  bool state;
  bool unstable_state;

  void setup(uint8_t pin) {
    this->pin = pin;
    state = unstable_state = 0;
    last_change_ms = millis();
  }

  bool read_state() {
    bool read_val = !(PINC & _BV(pin));

    if (read_val != unstable_state) {
#ifdef DIMMER_DEBBUG
      if (read_val == state) {
        Serial.print("unstable change  ");
        Serial.println(millis() - last_change_ms);
      }
#endif
      unstable_state = read_val;
      last_change_ms = millis();
    } else {
      if (read_val != state && (millis() - last_change_ms > DEBOUNCE_INTERVAL_MS)) {
        state = read_val;
#ifdef DIMMER_DEBBUG
        Serial.print("stable change ************  ");
        Serial.println(state?"ON":"OFF");
#endif
      }
    }
    return state;
  }
};

#endif
