#ifndef ARDUINO_H
#define ARDUINO_H

#define HIGH 1
#define LOW 0

typedef unsigned char byte;

unsigned long millis();
int clock_init();

void delay(unsigned long time);
float constrain(float x, float a, float b);

int digitalRead(int pin);
void digitalWrite(int pin, int val);
void analogWrite(int pin, int val);
void delayMicroseconds(unsigned long us);
#endif

/* vi: set et sw=2 sts=2: */
