#include "corobot_diagnostics/lcd.hpp"
#include <stdio.h>

using namespace std;

Lcd::Lcd()
{

}

Lcd::~Lcd()
{

}

void Lcd::write(string message) const
{
    printf("%s\n", message.c_str());
}

void Lcd::open() const
{

}

void Lcd::close() const
{

}
