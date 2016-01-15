/*
  This code contibuted by Triffid_Hunter and modified by Kliment
  why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
*/

#ifndef _FASTIO_ARDUINO_H
#define _FASTIO_ARDUINO_H

//#include <avr/io.h>

/*
  utility functions
*/

void WRITE(unsigned IO, int v);
int READ(unsigned IO); 

void SET_INPUT(unsigned IO);
void SET_OUTPUT(unsigned IO);

#endif
