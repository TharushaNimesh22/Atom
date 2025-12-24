#include "display.h"

Display::Display()
  :_display(128, 64, &Wire)
{  
}

///////////////////////
void Display::init(){
    _display.begin(0x3C, true);
    _display.clearDisplay();
    _display.setTextColor(SH110X_WHITE);
}

//////////////////////////////
void Display::clearDisplay(){
  _display.clearDisplay();
}

// Write to display
void Display::write(int textSize, int cursorX, int cursorY, char text[]){
  _display.setTextSize(textSize);
  _display.setCursor(cursorX, cursorY);
  _display.println(text);

  _display.display();
}





