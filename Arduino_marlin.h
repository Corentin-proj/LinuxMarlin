#ifndef ARDUINO_H
#define ARDUINO_H

#define HIGH 1
#define LOW 0

typedef unsigned char byte;

int clock_init();

#if MRAA == 1
unsigned long millis();
int digitalRead(int pin);
void digitalWrite(int pin, int val);
void delay(unsigned long time);
void delayMicroseconds(unsigned long us);
void analogWrite(int pin, int val);
float constrain(float x, float a, float b);

#endif
#endif

/* vi: set et sw=2 sts=2: */
