#ifndef PINS_H
#define PINS_H

#define X_MS1_PIN -1
#define X_MS2_PIN -1
#define Y_MS1_PIN -1
#define Y_MS2_PIN -1
#define Z_MS1_PIN -1
#define Z_MS2_PIN -1
#define E0_MS1_PIN -1
#define E0_MS2_PIN -1
#define E1_MS1_PIN -1
#define E1_MS2_PIN -1
#define DIGIPOTSS_PIN -1

/****************************************************************************************
* Teensylu 0.7 / Printrboard pin assignments (AT90USB1286)
* Requires the Teensyduino software with Teensy++ 2.0 selected in Arduino IDE!
  http://www.pjrc.com/teensy/teensyduino.html
* See http://reprap.org/wiki/Printrboard for more info
****************************************************************************************/
#if MOTHERBOARD == 84 /* --TOM--: MinnowBoard */
#define KNOWN_BOARD 1
#define AT90USB 1286  // Disable MarlinSerial etc.

#define __AVR_AT90USB1286__

#ifndef __AVR_AT90USB1286__
#error Oops!  Make sure you have 'Teensy++ 2.0' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH        true

//Disable JTAG pins so they can be used for the Extrudrboard
#define DISABLE_JTAG       true

#define X_STEP_PIN          3
#define X_DIR_PIN           4
#define X_ENABLE_PIN     		7 

#define Y_STEP_PIN          5
#define Y_DIR_PIN           8
#define Y_ENABLE_PIN        12

#define Z_STEP_PIN          6
#define Z_DIR_PIN           13
#define Z_ENABLE_PIN        14

#define E0_STEP_PIN         9
#define E0_DIR_PIN          15
#define E0_ENABLE_PIN     	16

#define HEATER_0_PIN       -1  // Extruder
#define HEATER_1_PIN       -1
#define HEATER_2_PIN       -1
#define HEATER_BED_PIN     -1  // Bed
#define FAN_PIN        		 -1  // Fan

#define X_STOP_PIN         17
#define Y_STOP_PIN         18
#define Z_STOP_PIN         19
//#define TEMP_0_PIN          1  // Extruder / Analog pin numbering
//#define TEMP_BED_PIN        0  // Bed / Analog pin numbering
//#define SDSS               20

#define SDPOWER            -1
#define LED_PIN            -1
#define PS_ON_PIN          -1
#define KILL_PIN           -1
#define ALARM_PIN          -1

#define DAC_STEPPER_CURRENT
#define DAC_STEPPER_ADDRESS	0
#define DAC_STEPPER_ORDER 	{3,2,1,0}
#define DAC_STEPPER_MAX 	3520
#define DAC_STEPPER_VREF 	1 //internal Vref, gain 1x = 2.048V
#define DAC_STEPPER_GAIN	0

#endif  

#ifndef KNOWN_BOARD
#error Unknown MOTHERBOARD value in configuration.h
#endif

//List of pins which to ignore when asked to change by gcode, 0 and 1 are RX and TX, do not mess with those!
#define _E0_PINS E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, HEATER_0_PIN,
#if EXTRUDERS > 1
  #define _E1_PINS E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN, HEATER_1_PIN,
#else
  #define _E1_PINS
#endif
#if EXTRUDERS > 2
  #define _E2_PINS E2_STEP_PIN, E2_DIR_PIN, E2_ENABLE_PIN, HEATER_2_PIN,
#else
  #define _E2_PINS
#endif

#ifdef X_STOP_PIN
  #if X_HOME_DIR < 0
    #define X_MIN_PIN X_STOP_PIN
    #define X_MAX_PIN -1
  #else
    #define X_MIN_PIN -1
    #define X_MAX_PIN X_STOP_PIN
  #endif
#endif

#ifdef Y_STOP_PIN
  #if Y_HOME_DIR < 0
    #define Y_MIN_PIN Y_STOP_PIN
    #define Y_MAX_PIN -1
  #else
    #define Y_MIN_PIN -1
    #define Y_MAX_PIN Y_STOP_PIN
  #endif
#endif

#ifdef Z_STOP_PIN
  #if Z_HOME_DIR < 0
    #define Z_MIN_PIN Z_STOP_PIN
    #define Z_MAX_PIN -1
  #else
    #define Z_MIN_PIN -1
    #define Z_MAX_PIN Z_STOP_PIN
  #endif
#endif

#ifdef DISABLE_MAX_ENDSTOPS
#define X_MAX_PIN          -1
#define Y_MAX_PIN          -1
#define Z_MAX_PIN          -1
#endif

#ifdef DISABLE_MIN_ENDSTOPS
#define X_MIN_PIN          -1
#define Y_MIN_PIN          -1
#define Z_MIN_PIN          -1
#endif

#endif

