#ifndef UTILS_H
#define UTILS_H

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

#endif

