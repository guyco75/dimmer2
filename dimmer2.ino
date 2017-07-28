
#define DEBOUNCE_INTERVAL_UNPRESSED_MS (100)  // Unstable/debounce duration when a button is pressed
#define DEBOUNCE_INTERVAL_PRESSED_MS (50)     // Unstable/debounce duration when a button is released

#define BTN_LONG_PRESS_THRESHOLD_MS (500)     // Below that it's considered a SHORT press; above it a LONG press

#define LOOP_DURATION (40)

//#define DIMMER_DEBBUG 1

#ifdef DIMMER_DEBBUG

#define assert(test, str1, str2) \
do { \
  if (!(test)) { \
    Serial.println("*** ASSERT ***"); \
    Serial.println(str1); \
    Serial.println(str2); \
    while (1) {;} \
  } \
} while (0)

#define SERIAL_PRINT_VAR(_x) do { Serial.print(#_x " = "); Serial.println(_x); } while (0);

#else

#define assert(test, str1, str2)
#define SERIAL_PRINT_VAR(_x)

#endif

struct button {
  unsigned long last_change_ms;
  uint16_t debounce_interval_ms;
  uint8_t pin;
  bool state;
  bool unstable_state;

  void setup(uint8_t pin) {
    this->pin = pin;
    pinMode(pin, INPUT_PULLUP);
    state = unstable_state = 0;
    debounce_interval_ms = DEBOUNCE_INTERVAL_UNPRESSED_MS;
    last_change_ms = millis();
  }

  bool read_state() {
    bool readVal = !digitalRead(pin);

    if (readVal != unstable_state) {
#ifdef DIMMER_DEBBUG
      Serial.println("unstable change");
#endif
      unstable_state = readVal;
      last_change_ms = millis();
    } else {
      if (readVal != state && (millis() - last_change_ms > debounce_interval_ms)) {
        state = readVal;
        debounce_interval_ms = state?DEBOUNCE_INTERVAL_PRESSED_MS:DEBOUNCE_INTERVAL_UNPRESSED_MS;
#ifdef DIMMER_DEBBUG
        Serial.print("stable change ************  ");
        Serial.println(state?"ON":"OFF");
#endif
      }
    }
    return state;
  }
};


enum light_state {
  LIGHT_ST_NO_ACTION,
  LIGHT_ST_BTN_ARMED,
  LIGHT_ST_SHORT_PRESS,
  LIGHT_ST_LONG_PRESS,
  //LIGHT_ST_STOP,
  LIGHT_ST_MAX
};

#ifdef DIMMER_DEBBUG
static const char *button_state_names[] = {
  "NO_ACTION",
  "BTN_ARMED",
  "SHORT_PRESS",
  "LONG_PRESS",
  //"STOP"
};
#endif

enum light_change_speed {
  LIGHT_CHANGE_VERY_SLOW,
  LIGHT_CHANGE_SLOW,
  LIGHT_CHANGE_FAST,
  LIGHT_CHANGE_IMMEDIATE,
  LIGHT_CHANGE_MAX
};  

static uint8_t light_change_speed_factor[LIGHT_CHANGE_MAX] = {
  8, //LIGHT_CHANGE_VERY_SLOW
  4, //LIGHT_CHANGE_SLOW
  3, //LIGHT_CHANGE_FAST
  1, //LIGHT_CHANGE_IMMEDIATE
};

enum dimmer_direction {
  DIMMER_DIR_UP,
  DIMMER_DIR_DOWN,
  DIMMER_DIR_NONE,
  DIMMER_DIR_MAX,
};

struct light {
  button btn_up;
  button btn_down;

  uint16_t brightness;
  uint16_t target_brightness;
  //uint16_t orig_brightness; // remember the brightness when going to IDLE (PIR)
  uint16_t min_brightness;
  uint16_t max_brightness;
  //bool turnOffWhenIdle; // will not turn on back automatically
  enum light_state state;
  unsigned long action_start_time;
  //unsigned long idleTime;
  //struct pirSensor pirSensor;
  enum dimmer_direction dimmer_direction;

  void set_dimmer(int b) {
#ifdef DIMMER_DEBBUG
    Serial.print("\t\t\t\t\t\t");
    Serial.println(b);
#endif
    //OCR1A = b;
  }

  void setup(uint8_t up_pin, uint8_t down_pin, uint16_t minb, uint16_t maxb) {
    btn_up.setup(up_pin);
    btn_down.setup(down_pin);
    min_brightness = minb;
    max_brightness = maxb;

    dimmer_direction = DIMMER_DIR_NONE;
  }

  void change_state(enum light_state st) {
#ifdef DIMMER_DEBBUG
    Serial.print(millis());
    Serial.print("\t\t");
    Serial.print(button_state_names[state]);
    Serial.print(" --> ");
    Serial.println(button_state_names[st]);
#endif
    state = st;
  }

  //return true if we're done
  bool step_to_target_brightness(enum light_change_speed s) {
    uint8_t speed_factor = light_change_speed_factor[s];
    uint16_t brightness_step = (brightness>>speed_factor) + 1;
    
    if (brightness > target_brightness) {
      brightness = max(target_brightness, brightness - brightness_step); // going down
    } else {
      brightness = min(target_brightness, brightness + brightness_step); // going up
    }
    set_dimmer(brightness);
    return brightness == target_brightness;
  }

  // return true if active
  bool fsm() {
    bool button_pressed_up   = btn_up.read_state();
    bool button_pressed_down = btn_down.read_state();
    bool button_pressed_any  = button_pressed_up || button_pressed_down;

    //SERIAL_PRINT_VAR(button_pressed_up);
    //SERIAL_PRINT_VAR(button_pressed_down);
    //SERIAL_PRINT_VAR(button_pressed_any);

    if (state == LIGHT_ST_NO_ACTION && !button_pressed_any) {
      return false; // nothing to do
    }

    // Note: clicking a button is ignored if/while the other one is already help
    // However we should take care of a case that the first one was released and the second one pressed (at the same time or before):

    if (dimmer_direction == DIMMER_DIR_UP) {
      if (!button_pressed_up && button_pressed_down) {
        // direction change - reset
        dimmer_direction = DIMMER_DIR_DOWN;
        change_state(LIGHT_ST_NO_ACTION);
      }
    }

    if (dimmer_direction == DIMMER_DIR_DOWN) {
      if (!button_pressed_down && button_pressed_up) {
        // direction change - reset
        dimmer_direction = DIMMER_DIR_UP;
        change_state(LIGHT_ST_NO_ACTION);
      }
    }

    switch (state) {
    case LIGHT_ST_NO_ACTION:
      assert(button_pressed_any, "in LIGHT_ST_NO_ACTION but no button was pressed", "");

      if (button_pressed_up) {
        dimmer_direction = DIMMER_DIR_UP;
        target_brightness = max_brightness;
      } else {
        dimmer_direction = DIMMER_DIR_DOWN;
        target_brightness = min_brightness;
      }

      action_start_time = millis();
      change_state(LIGHT_ST_BTN_ARMED);
      break;

    case LIGHT_ST_BTN_ARMED:
      if (!button_pressed_any) { // short click
        change_state(LIGHT_ST_SHORT_PRESS);
      } else {
        if ((millis() - action_start_time) >= BTN_LONG_PRESS_THRESHOLD_MS) { // long click
          step_to_target_brightness(LIGHT_CHANGE_SLOW);
          change_state(LIGHT_ST_LONG_PRESS);
        }
      }
      break;

    case LIGHT_ST_SHORT_PRESS:
      if (!button_pressed_any) {                             // continue to target_trightness
        if (step_to_target_brightness(LIGHT_CHANGE_FAST)) {
          change_state(LIGHT_ST_NO_ACTION);                  // we're done
        }
      } else {
        change_state(LIGHT_ST_NO_ACTION);                    // interrupted - go through the fsm to react
      }
      break;

    case LIGHT_ST_LONG_PRESS:
      if (!button_pressed_any) {
        change_state(LIGHT_ST_NO_ACTION);                    // we're done
      } else {
        step_to_target_brightness(LIGHT_CHANGE_SLOW);        // continue to target brightness (even if we already reached it)
      }                                                      // wait until the user released the button
      break;

    default:
      break;
    }
    return true; // we're probably not idle
  }
};

static light l;

void setup() {
  Serial.begin(9600);
  Serial.println("--Ready--");

  l.setup(5, 6, 0, 0x3ff);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
}

void loop() {
  unsigned long m = millis();
  l.fsm();
  //digitalWrite(13, LOW);
  while(millis() < m + LOOP_DURATION);
}

