#ifndef DISPLAY_H
#define DISPLAY_H

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
// #include <string>

class Display{
    public:
      Display();

      void init();
      void clearDisplay();
      void write(int textSize, int cursorX, int cursorY, char text[]);

    private:
      Adafruit_SH1106G _display;

};

#endif