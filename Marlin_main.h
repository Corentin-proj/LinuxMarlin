#ifndef _GPIO_TEST_H
#define _GPIO_TEST_H

#include "Marlin.h"
#include "Configuration.h"
#include "ConfigurationStore.h"
#include "Arduino_marlin.h"
#include "temperature.h"
#include "planner.h"
#include "stepper.h"
#include "language.h"
#include "Arduino.h"
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include <fcntl.h>
#include <sys/stat.h>

//===========================================================================
//=============================public variables=============================
//===========================================================================
// none of variables here need to be protected by lock coz they won't be
// accessed from except the main thread
int extrudemultiply=100; //100->1 200->2
float add_homeing[3]={0,0,0};
bool axis_known_position[3] = {false, false, false};
//--TOM--: the active extruder is always #0 coz that's the only one we've got
char active_extruder = 0;
int fanSpeed=0;

float base_min_pos[3] = { X_MIN_POS_DEFAULT, Y_MIN_POS_DEFAULT, Z_MIN_POS_DEFAULT };
float base_max_pos[3] = { X_MAX_POS_DEFAULT, Y_MAX_POS_DEFAULT, Z_MAX_POS_DEFAULT };
#ifdef ENABLE_AUTO_BED_LEVELING
float bed_level_probe_offset[3] = {X_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT,
	Y_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT, -Z_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT};
#endif

//===========================================================================
//=============================private variables=============================
//===========================================================================
static float homing_feedrate[] = HOMING_FEEDRATE;
static bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
static int feedmultiply=100; //100->1 200->2
static int saved_feedmultiply;
static float min_pos[3] = { X_MIN_POS_DEFAULT, Y_MIN_POS_DEFAULT, Z_MIN_POS_DEFAULT };
static float max_pos[3] = { X_MAX_POS_DEFAULT, Y_MAX_POS_DEFAULT, Z_MAX_POS_DEFAULT };
static float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
static float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};
static const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float offset[3] = {0.0, 0.0, 0.0};
static bool home_all_axis = true;
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;
static bool relative_mode = false;  //Determines Absolute or Relative Coordinates

static char cmdbuffer[MAX_CMD_SIZE];
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;

static unsigned long starttime=0;
static unsigned long stoptime=0;

static uint8_t tmp_extruder;

static bool target_direction;
static bool Stopped = false;

static char *file_buf = NULL;
static int file_size;

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
static const type array##_P[3] =        \
    { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
static inline type array(int axis)          \
    { return array##_P[axis]; }

#define XYZ_DYN_FROM_CONFIG(type, array, CONFIG)	\
static inline type array(int axis)			\
    { type temp[3] = { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };\
      return temp[axis];}

XYZ_DYN_FROM_CONFIG(float, base_home_pos,   HOME_POS);
XYZ_DYN_FROM_CONFIG(float, max_length, MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

//===========================================================================
//=============================ROUTINES=============================
//===========================================================================
bool setTargetedHotend(int code);
float code_value(void);
long code_value_long(void);
bool code_seen(char code);
void ikill(void);
int marlin_main(int argc, char *argv[]);
int setup(char *path);
void loop(int fd);
static void run_z_probe(void);
static void clean_up_after_endstop_move(void);
#ifdef ENABLE_AUTO_BED_LEVELING
static void set_bed_level_equation(float z_at_xLeft_yFront, float z_at_xRight_yFront, float z_at_xLeft_yBack);
#endif
void do_blocking_move_to(float x, float y, float z);
void setup_for_endstop_move(void);
bool IsStopped(void);
void Stop(void);
bool get_command(void);
static void axis_is_at_home(int axis);
static void homeaxis(int axis);
void do_home(void);
#ifdef ENABLE_AUTO_BED_LEVELING
void do_auto_bed_leveling(void);
#endif
void do_set_position(void);
void set_temp_and_wait(void);
void stop_idle_hold(void);
void process_commands(void);
void get_coordinates(void);
void clamp_to_software_endstops(float target[3]);
void prepare_move(void);
void manage_inactivity(void);
bool setTargetedHotend(int code);

#endif
