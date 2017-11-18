#ifndef SERIAL_PARSER_H
#define SERIAL_PARSER_H

struct SerialParser {
  String rxCmdStr;
  uint16_t maxStrLen;
  uint16_t a;
  uint16_t b;
  bool inSync;

  SerialParser(uint16_t maxLen) : rxCmdStr(maxLen), maxStrLen(maxLen), inSync(false)
  {
  }

  inline String getNextToken() {
    String ret;

    b = rxCmdStr.indexOf(',', a);
    if (b < 1)
      return "";

    ret = rxCmdStr.substring(a,b);
    a = b+1;
    return ret;
  }

  inline boolean verifyEnding() {
    return (a == rxCmdStr.length());
  }

  //returns true if a command is available
  bool processSerial() {
    while (Serial.available()) {
      char c = Serial.read();

      if (c == '$') {
        rxCmdStr = "";
        a = b = 0;
        inSync = true;
        continue;
      }

      if (inSync) {
        if (c == '#') {
          inSync = false;
          return true;
        } else {
          if (rxCmdStr.length() < maxStrLen-1) {
            rxCmdStr += c;
          } else {
            inSync = false;
          }
        }
      }
    }
    return false;
  }
};

#endif

