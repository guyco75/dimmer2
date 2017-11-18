#include "serial_parser.h"
#include "button.h"
#include "dimmer.h"

#define SCALE (1)  // for debug
#define TIMER_PERIOD (20 * SCALE) // 20us
#define ZC_PERIOD (10000ULL * SCALE) // 10ms (2x50Hz) // TODO: move the UUL to scale (and have it UL)

#define LOOP_DURATION (50) //ms

//#define DIMMER_DEBBUG 1

#ifdef DIMMER_DEBBUG

struct statistics {
  uint32_t last_zero_cross_delta_us;
  uint32_t last_zero_cross_duration_us;
  uint32_t last_timer1_overflow_delta_us;
  uint32_t last_timer1_overflow_duration_us;
};
volatile statistics stat;

#endif

/*****************************************/

volatile uint32_t timer_cnt = 0;
void timer_setup(uint8_t zc_pin) {
  TCCR1B = _BV(WGM13);        // set mode as phase and frequency correct pwm, stop the timer
  TCCR1A = 0;                 // clear control register A 
  ICR1 = (F_CPU / 2000000) * TIMER_PERIOD;
  TIMSK1 = _BV(TOIE1);

  // must be pings 2 or 3
  pinMode(zc_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(zc_pin), zero_crosss_int, RISING);
}

void zero_crosss_int() {
#ifdef DIMMER_DEBBUG
  static uint32_t last_zc = 0;
  uint32_t cur_zc = TCNT0;
#endif

  PORTD = 0; // clear all light control output pins

  for (int i=0; i<LIGHT_ARR_SIZE; ++i) {
    light *l = &light_arr[i];

    //CSRdigitalWrite(l->light_pin, LOW);

    if (l->brightness == l->min_brightness) {
      l->shadow.timer_cnt_on = 0;
      l->shadow.timer_cnt_off = 0;
    } else {
      l->shadow.timer_cnt_on = l->brightness;
      l->shadow.timer_cnt_off = (l->dimmer_type == DIMMER_TYPE_TRIAC) ? l->brightness+5 : 0;
    }
  }

  timer_cnt = 0;
  TCNT1 = 1; // 0 will cause an immediate interrupt. Skip the first cycle (AC is anyway low for a while)
  TCCR1B = _BV(WGM13) | _BV(CS10); // enable the timer (highest resolution)

#ifdef DIMMER_DEBBUG
  stat.last_zero_cross_duration_us = TCNT0 - cur_zc;
  stat.last_zero_cross_delta_us = cur_zc - last_zc;
  last_zc = cur_zc;
#endif
}

ISR(TIMER1_OVF_vect) {
  if (timer_cnt == 0) // skip the first cycle (it may be a late/shorter one)
    goto out;         // NOTE: the code below assumes that timer_cnt is not zero!

  for (int i=0; i<LIGHT_ARR_SIZE; ++i) {
    light *l = &light_arr[i];
    if (timer_cnt == l->shadow.timer_cnt_on) {
      PORTD |= _BV(3);
    }
    if (timer_cnt == l->shadow.timer_cnt_off) {
      PORTD &= ~_BV(3);
    }
  }
out:
  timer_cnt++;
}

/*****************************************/

SerialParser serParser(128);

void handleSerialCmd() {
  int dimmer, br;

  dimmer = serParser.getNextToken().toInt();
  if (dimmer < 0 || LIGHT_ARR_SIZE <= dimmer) {Serial.println("${\"status\":\"ERR dimmer\"}#");return;}

  br = serParser.getNextToken().toInt();
  if (br < 0 || 0x3ff < br) {Serial.println("${\"status\":\"ERR brightness\"}#");return;}

  // TODO: return (ignore) if not idle
  light_arr[dimmer].target_brightness = br;
  light_arr[dimmer].change_state(LIGHT_ST_SHORT_PRESS);
}

unsigned long m;
void setup() {
  Serial.begin(57600);
  Serial.println("--Ready--");

  light_arr[0].setup(0, 1, 3, 450, 20,   DIMMER_TYPE_TRIAC);
/*
  light_arr[1].setup(2, 3, 4, 800, 100,   DIMMER_TYPE_TRIAC);
  light_arr[2].setup(4, 5, 5, 0,   0x3ff, DIMMER_TYPE_PWM);
  light_arr[3].setup(6, 7, 6, 0,   0x3ff, DIMMER_TYPE_PWM);
*/
#ifdef DIMMER_DEBBUG
  memset((void *)&stat, 0, sizeof(stat));
#endif

  DDRC = 0;    // set all analog pins to INPUT
  PORTC = 0xFF; // pull up

  m = millis();

  delay(10);       //TODO
  timer_setup(2);  //TODO
}

void loop() {
  for (int i=0; i<LIGHT_ARR_SIZE; ++i) {
    light_arr[i].fsm();
  }

  if (serParser.processSerial()) {
    handleSerialCmd();
  }

  while(millis() - m < LOOP_DURATION);
  m = millis();
}

