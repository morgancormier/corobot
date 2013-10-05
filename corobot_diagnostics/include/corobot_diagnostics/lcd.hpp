#ifndef LCD_H
#define LCD_H

#include <string>
#include <stdio.h>

class Lcd
// Interface to the LCD display
{
private:

public:
Lcd();
~Lcd();
void write(std::string message) const; //write a message on the lcd

};




#endif
