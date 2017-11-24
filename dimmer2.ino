//#define DIMMER_DEBBUG 1
//#define SIMULATE_ZC

#include "serial_parser.h"
#include "button.h"
#include "dimmer.h"

#define SCALE (1)  // for debug
#define TIMER_PERIOD (20 * SCALE) // 20us
#define ZC_PERIOD (10000ULL * SCALE) // 10ms (2x50Hz) // TODO: move the UUL to scale (and have it UL)

#ifdef DIMMER_DEBBUG

struct statistics {
  uint32_t last_zero_cross_delta_us;
  uint32_t last_zero_cross_duration_us;
  uint32_t last_timer1_overflow_delta_us;
  uint32_t last_timer1_overflow_duration_us;
};
volatile statistics stat;

#endif

#define LIGHT_ARR_SIZE (1)
static light light_arr[LIGHT_ARR_SIZE];

/*****************************************/

volatile uint32_t timer_cnt = 0;

void timer_setup() {
  TCCR1B = _BV(WGM13) | _BV(CS10); // enable the timer (highest resolution)
  TCCR1A = 0;                 // clear control register A 
  ICR1 = (F_CPU / 2000000) * TIMER_PERIOD;
  TIMSK1 = 0;                 // keep the interrupt disabled
}

void zero_crosss_int() {
#ifdef DIMMER_DEBBUG
  static uint32_t last_zc = 0;
  uint32_t cur_zc = TCNT0;
#endif

  timer_cnt = 0;
  TCNT1 = 1; // 0 will cause an immediate interrupt. Skip the first cycle (AC is anyway low for a while)
  TIMSK1 = _BV(TOIE1);             // enable the interrupt

#ifdef DIMMER_DEBBUG
  stat.last_zero_cross_duration_us = TCNT0 - cur_zc;
  stat.last_zero_cross_delta_us = cur_zc - last_zc;
  last_zc = cur_zc;
#endif
}

ISR(TIMER1_OVF_vect) {
  if (timer_cnt == 0) // skip the first cycle (it may be a late/shorter one)
    goto out;         // NOTE: the code below assumes that timer_cnt is not zero!

  if (timer_cnt >= 460) {

    TIMSK1 = 0; // disable the timer interrupt

    // prepare for next time
    for (int i=0; i<LIGHT_ARR_SIZE; ++i) {
      light *l = &light_arr[i];
      l->fsm();

      //TODO: replace with on/off flag
      if (l->brightness == l->min_brightness) {
        l->shadow.timer_cnt_on = 0;
        l->shadow.timer_cnt_off = 0;
      } else {
        l->shadow.timer_cnt_on = l->brightness;
        l->shadow.timer_cnt_off = (l->dimmer_type == DIMMER_TYPE_TRIAC) ? l->brightness+5 : 0;
      }
    }
    return;
  }

  for (int i=0; i<LIGHT_ARR_SIZE; ++i) {
    light *l = &light_arr[i];
    if (timer_cnt == l->shadow.timer_cnt_on) {
      *l->light_port |= l->light_port_bit;
    }
    if (timer_cnt == l->shadow.timer_cnt_off) {
      *l->light_port &= ~l->light_port_bit;
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

  light *l = &light_arr[dimmer];

  br = serParser.getNextToken().toInt();
  if (br < l->max_brightness || l->min_brightness < br) {Serial.println("${\"status\":\"ERR brightness\"}#");return;}

  // TODO: disable interrupts
  l->target_brightness = br;
  l->change_state(LIGHT_ST_SHORT_PRESS);
}

unsigned long m;
void setup() {
  Serial.begin(57600);
  Serial.println("--Ready--");

  light_arr[0].setup(0, 1, &PORTD, _BV(3), 450, 20,   DIMMER_TYPE_TRIAC);
/*
  light_arr[1].setup(2, 3, 4, 800, 100,   DIMMER_TYPE_TRIAC);
  light_arr[2].setup(4, 5, 5, 0,   0x3ff, DIMMER_TYPE_PWM);
  light_arr[3].setup(6, 7, 6, 0,   0x3ff, DIMMER_TYPE_PWM);
*/
  pinMode(3, OUTPUT);

#ifdef DIMMER_DEBBUG
  memset((void *)&stat, 0, sizeof(stat));
#endif

  DDRC = 0;    // set all analog pins to INPUT
  PORTC = 0xFF; // pull up

  delay(10);       //TODO
  timer_setup();   //TODO

#ifndef SIMULATE_ZC
  // must be pings 2 or 3
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), zero_crosss_int, RISING);
#endif
  m = millis();
}

#ifdef SIMULATE_ZC
  #define LOOP_DURATION (10) //ms
#else
  #define LOOP_DURATION (100) //ms
#endif

void loop() {

  if (serParser.processSerial()) {
    handleSerialCmd();
  }

#ifdef SIMULATE_ZC
  zero_crosss_int();
#endif
  while(millis() - m < LOOP_DURATION);
  m = millis();
}

