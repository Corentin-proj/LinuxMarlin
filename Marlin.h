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

#include "Arduino.h"
#include "fastio.h"
#include "Configuration.h"
#include "pins.h"
#include <signal.h>

/* only support this frequency value for now */
#define F_CPU 16000000

#define SERIAL_PROTOCOL(x) printf("%s\n", x)
#define SERIAL_PROTOCOL_F(x,y) printf("%f\n", x)
#define SERIAL_PROTOCOLPGM(x) printf("%s\n", x)
#define SERIAL_PROTOCOLLN(x) printf("%s\n", x)
#define SERIAL_PROTOCOLLNPGM(x) printf("%s\n", x)

#define errExit(msg) 	do { perror(msg); exit(EXIT_FAILURE); \
										 	} while (0)

#ifdef DEBUG
#define DEBUG_PRINT(...) do{ fprintf( stderr, __VA_ARGS__ ); } while( false )
#else
#define DEBUG_PRINT(...) do{ } while ( false )
#endif
#define SERIAL_ECHO_START 
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHOPAIR(name,value) SERIAL_PROTOCOL(name)

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

sigset_t global_interrupt, old_global_interrupt;
extern inline void cli();
extern inline void sei();
unsigned char SREG;

#if defined(DUAL_X_CARRIAGE) && defined(X_ENABLE_PIN) && X_ENABLE_PIN > -1 \
    && defined(X2_ENABLE_PIN) && X2_ENABLE_PIN > -1
  #define  enable_x() ;//do { WRITE(X_ENABLE_PIN, X_ENABLE_ON); WRITE(X2_ENABLE_PIN, X_ENABLE_ON); } while (0)
  #define disable_x() ;//do { WRITE(X_ENABLE_PIN,!X_ENABLE_ON); WRITE(X2_ENABLE_PIN,!X_ENABLE_ON); axis_known_position[X_AXIS] = false; } while (0)
#elif defined(X_ENABLE_PIN) && X_ENABLE_PIN > -1
  #define  enable_x() ;//WRITE(X_ENABLE_PIN, X_ENABLE_ON)
  #define disable_x() ;//{ WRITE(X_ENABLE_PIN,!X_ENABLE_ON); axis_known_position[X_AXIS] = false; }
#else
  #define enable_x() ;
  #define disable_x() ;
#endif

#if defined(Y_ENABLE_PIN) && Y_ENABLE_PIN > -1
  #ifdef Y_DUAL_STEPPER_DRIVERS
    #define  enable_y() ;//{ WRITE(Y_ENABLE_PIN, Y_ENABLE_ON); WRITE(Y2_ENABLE_PIN,  Y_ENABLE_ON); }
    #define disable_y() ;//{ WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON); WRITE(Y2_ENABLE_PIN, !Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }
  #else
    #define  enable_y() ;//WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
    #define disable_y() ;//{ WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }
  #endif
#else
  #define enable_y() ;
  #define disable_y() ;
#endif

//TODO
#if defined(Z_ENABLE_PIN) && Z_ENABLE_PIN > -1
  #ifdef Z_DUAL_STEPPER_DRIVERS
    #define  enable_z() ;//{ WRITE(Z_ENABLE_PIN, Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN, Z_ENABLE_ON); }
    #define disable_z() ;//{ WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN,!Z_ENABLE_ON); axis_known_position[Z_AXIS] = false; }
  #else
    #define  enable_z() ;//WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
    #define disable_z() ;//{ WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON); axis_known_position[Z_AXIS] = false; }
  #endif
#else
  #define enable_z() ;
  #define disable_z() ;
#endif

#if defined(E0_ENABLE_PIN) && (E0_ENABLE_PIN > -1)
  #define enable_e0() ;//WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e0() ;//WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)
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


#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

extern int extrudemultiply;
extern int fanSpeed;
extern char active_extruder;

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
