#ifndef _GPIO_TEST_H
#define _GPIO_TEST_H

/* standard headers */
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

/* mraa header */
#include "mraa/gpio.h"

/*arduino headers*/
#include "Arduino.h"

/*header*/
#include "fastio.h"

#ifdef __cplusplus
extern "C" {
#endif

void sig_handler(int signum);
int main_gpio(void);

#ifdef __cplusplus
}
#endif

#endif
