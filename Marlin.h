// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL

#ifndef MARLIN_H
#define MARLIN_H

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include <unistd.h>

// #include <util/delay.h>
// #include <avr/pgmspace.h>
// #include <avr/eeprom.h>
// #include <avr/interrupt.h>

#include "Arduino_marlin.h"
#include "fastio.h"
#include "Configuration.h"
#include "pins.h"
#include <signal.h>

//CPU frequency in Hz
//#define F_CPU 16000000
#define F_CPU 400000000 //400Mhz
#define FASTIO 0

#define HAL_TIMER_RATE         (F_CPU / 4)  // frequency of timers peripherals
#define STEPPER_TIMER_RATE     HAL_TIMER_RATE   // frequency of stepper timer (HAL_TIMER_RATE / STEPPER_TIMER_PRESCALE)
#define STEPPER_TIMER_TICKS_PER_US ((STEPPER_TIMER_RATE) / 1000000) // stepper timer ticks per µs
#define STEPPER_TIMER_PRESCALE (CYCLES_PER_MICROSECOND / STEPPER_TIMER_TICKS_PER_US)

#define PULSE_TIMER_RATE       STEPPER_TIMER_RATE   // frequency of pulse timer
#define PULSE_TIMER_PRESCALE   STEPPER_TIMER_PRESCALE
#define PULSE_TIMER_TICKS_PER_US STEPPER_TIMER_TICKS_PER_US

#define ECHO_FLOAT(x) printf("%lf", x)
#define ECHO_STRING(x) printf("%s", x)
#define ECHO_DECIMAL(x) printf("%lu", x)
#define ECHOPAIR_L(name,value) printf("%s: %lu", name, value)
#define ECHOPAIR_F(name,value) printf("%s: %lf", name, value)

#define errExit(msg) 	do { perror(msg); exit(EXIT_FAILURE); \
										 	} while (0)

#ifdef DEBUG
#define DEBUG_PRINT(...) do{ fprintf( stderr, __VA_ARGS__ ); } while( false )
#else
#define DEBUG_PRINT(...) do{ } while ( false )
#endif

enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};

int setup(char *path);
void loop();
bool get_command();
void process_commands();
void get_coordinates();
void prepare_move();
void clamp_to_software_endstops(float target[3]);
void ikill();
void manage_inactivity();
bool IsStopped();
void Stop();

//sigset_t global_interrupt, old_global_interrupt;
//extern inline void cli();
//extern inline void sei();
//unsigned char SREG;

#if defined(X_ENABLE_PIN) && X_ENABLE_PIN > -1
  #define  enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
  #define disable_x() { WRITE(X_ENABLE_PIN,!X_ENABLE_ON); axis_known_position[X_AXIS] = false; }
#else
  #define enable_x() ;
  #define disable_x() ;
#endif

#if defined(Y_ENABLE_PIN) && Y_ENABLE_PIN > -1
  #define  enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
  #define disable_y() { WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }
#else
  #define enable_y() ;
  #define disable_y() ;
#endif

#if defined(Z_ENABLE_PIN) && Z_ENABLE_PIN > -1
 #define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
 #define disable_z() { WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON); axis_known_position[Z_AXIS] = false; }
#else
  #define enable_z() ;
  #define disable_z() ;
#endif

#if defined(E0_ENABLE_PIN) && (E0_ENABLE_PIN > -1)
  #define enable_e0() WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e0() WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e0()  /* nothing */
  #define disable_e0() /* nothing */
#endif

#if (EXTRUDERS > 1) && defined(E1_ENABLE_PIN) && (E1_ENABLE_PIN > -1)
  #define enable_e1() ;//WRITE(E1_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e1() ;//WRITE(E1_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e1()  /* nothing */
  #define disable_e1() /* nothing */
#endif

#if (EXTRUDERS > 2) && defined(E2_ENABLE_PIN) && (E2_ENABLE_PIN > -1)
  #define enable_e2() ;//WRITE(E2_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e2() ;//WRITE(E2_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e2()  /* nothing */
  #define disable_e2() /* nothing */
#endif


/*
#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  cli();
  #define CRITICAL_SECTION_END    sei();
#endif //CRITICAL_SECTION_START
*/

extern int extrudemultiply;
extern int fanSpeed;
extern char active_extruder;
extern float add_homeing[3];
#ifdef ENABLE_AUTO_BED_LEVELING
extern float bed_level_probe_offset[3];
#endif
extern float base_min_pos[3];
extern float base_max_pos[3];
extern bool axis_known_position[3];

#ifdef FAN_SOFT_PWM
extern unsigned char fanSpeedSoftPwm;
#endif

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
     _a > _b ? _b : _a; })

#define square(x) (x*x)

#endif

/* vi: set et sw=2 sts=2: */
