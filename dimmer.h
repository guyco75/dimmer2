#ifndef DIMMER_H
#define DIMMER_H

#include "button.h"
#include "utils.h"

#define BTN_LONG_PRESS_THRESHOLD_MS (500)     // Below that it's considered a SHORT press; above it a LONG press

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
  LIGHT_CHANGE_MAX
};  

static uint8_t light_change_speed_factor[] = {
  10, //LIGHT_CHANGE_VERY_SLOW
  6, //LIGHT_CHANGE_SLOW
  4, //LIGHT_CHANGE_FAST
};

enum dimmer_direction {
  DIMMER_DIR_UP,
  DIMMER_DIR_DOWN,
  DIMMER_DIR_NONE,
  DIMMER_DIR_MAX,
};

enum dimmer_type {
  DIMMER_TYPE_PWM,
  DIMMER_TYPE_TRIAC,
  DIIMER_TYPE_MAX
};

struct light_shadow {
  uint32_t timer_cnt_on;
  uint32_t timer_cnt_off;
};

struct light {
  button btn_up;
  button btn_down;
  light_shadow shadow;

  int16_t brightness;
  int16_t target_brightness;
  //uint16_t orig_brightness; // remember the brightness when going to IDLE (PIR)
  int16_t min_brightness;
  int16_t max_brightness;
  //bool turnOffWhenIdle; // will not turn on back automatically
  enum light_state state;
  unsigned long action_start_time;
  //unsigned long idleTime;
  //struct pirSensor pirSensor;
  enum dimmer_direction dimmer_direction;
  enum dimmer_type dimmer_type;
  uint8_t light_pin;

  void setup(uint8_t up_pin, uint8_t down_pin, uint8_t l_pin, uint16_t minb, uint16_t maxb, enum dimmer_type d_type) {
    btn_up.setup(up_pin);
    btn_down.setup(down_pin);
    min_brightness = minb;
    max_brightness = maxb;
    dimmer_type = d_type;
    light_pin = l_pin;
    pinMode(light_pin, OUTPUT);

    state = LIGHT_ST_NO_ACTION;
    dimmer_direction = DIMMER_DIR_NONE;
    brightness = min_brightness;
    shadow.timer_cnt_on = 0;
    shadow.timer_cnt_off = 0;
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
    if (state == LIGHT_ST_NO_ACTION) {
      dimmer_direction = DIMMER_DIR_NONE;
    }
  }

  //return true if we're done
  bool step_to_target_brightness(enum light_change_speed s) {
    switch (dimmer_type) {
      case DIMMER_TYPE_PWM:   return step_to_target_brightness_pwm(s);
      case DIMMER_TYPE_TRIAC: return step_to_target_brightness_triac(s);
      default: assert(false, "invalid dimmer type", dimmer_type);
    }
    return true;
  }

  bool step_to_target_brightness_pwm(enum light_change_speed s) {
    int8_t speed_factor = light_change_speed_factor[s];
    int16_t brightness_step = (brightness>>speed_factor) + 1;
    
    if (brightness > target_brightness) {
      brightness = max(target_brightness, brightness - brightness_step); // going down
    } else {
      brightness = min(target_brightness, brightness + brightness_step); // going up
    }
#ifdef DIMMER_DEBBUG
    Serial.print("\t\t\t\t\t\t");
    Serial.println(brightness);
#endif
    analogWrite(9, brightness>>2);

    return brightness == target_brightness;
  }

  //return true if we're done
  bool step_to_target_brightness_triac(enum light_change_speed s) {
    int16_t brightness_step = (s+1)*3-2;//TODO
/*    Serial.println("-");
    Serial.println(brightness);
    Serial.print(" --- ");
    Serial.print(brightness_step);
    Serial.print(" --- ");
    Serial.println(brightness - brightness_step);*/

    if (brightness > target_brightness) {
      brightness = max(target_brightness, brightness - brightness_step); // going down
    } else {
      brightness = min(target_brightness, brightness + brightness_step); // going up
    }
/*#ifdef DIMMER_DEBBUG
    Serial.print("\t\t\t\t\t\t");
    Serial.print(id);
    Serial.print("   ");
    Serial.println(brightness);
#endif*/
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
        //dimmer_direction = DIMMER_DIR_DOWN;
        change_state(LIGHT_ST_NO_ACTION);
      }
    }

    if (dimmer_direction == DIMMER_DIR_DOWN) {
      if (!button_pressed_down && button_pressed_up) {
        // direction change - reset
        //dimmer_direction = DIMMER_DIR_UP;
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

#define LIGHT_ARR_SIZE (1)
static light light_arr[LIGHT_ARR_SIZE];

/*****************************************/

#endif

