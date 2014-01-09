#include "corobot_diagnostics/lcd.hpp"
//#include <usblcd.h>

using namespace std;

/** I hate to have to declare this as a global variable but due to how the lcd driver has been written, I have no choice.
 *  The reason is that in usblcd.h a global variable has been defined. This variable get declared multiple times if
 *  usblcd.h is included in the lcd.hpp and if another file include lcd.hpp (diagnostic.h does).
 *  Therefore, I did not find any other solution, except rewritting the driver, than declaring it as global variable
 */
//usblcd_operations *mylcd; // our lcd

Lcd::Lcd()
{
    /* init hid device and usblcd_operations structure */
 //   mylcd = new usblcd_operations();

    /* init the USB LCD */
 //   mylcd->init(mylcd);

    /* sets backlight to on */
  //  mylcd->backlight(mylcd,1);
}

Lcd::~Lcd()
{
    /* close the USB LCD device */
  //  mylcd->close(mylcd);

  //  delete mylcd;
}

void Lcd::write(string message) const
{
  //  mylcd->clear(mylcd); // clear the lcd screen
  //  mylcd->settext(mylcd, 0, 0, (char*) message.c_str()); // set text
}
