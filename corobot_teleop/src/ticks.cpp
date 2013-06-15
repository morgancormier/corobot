/*!
  \file
  \brief タイムスタンプ取得関数

  \author Satofumi KAMIMURA

  $Id: ticks.c 1734 2010-03-06 01:32:54Z satofumi $
*/

#include "ticks.h"
#include "urg_ticks.h"


long ticks(void)
{
  return urg_ticks();
}
