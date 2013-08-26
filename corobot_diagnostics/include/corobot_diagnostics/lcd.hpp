#ifndef LCD_H
#define LCD_H

#include <string>
class Lcd
{

private:


public:
Lcd();
~Lcd();
void open() const;
void close() const;
void write(std::string message) const;

};




#endif
