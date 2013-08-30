#ifndef LCD_H
#define LCD_H

#include <string>
#include <stdio.h>

class Lcd
// Interface to the LCD display
{
private:
bool isOpened; // tell if the lcd has been openned or not

public:
Lcd();
~Lcd();
void open(); // open the lcd. Necessary before any use
void close(); // Close the lcd. If not called the destructor will take care of calling it.
void write(std::string message) const; //write a message on the lcd

};




#endif
