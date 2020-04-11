/* -*- c++ -*- */

/*
    Reprap firmware based on Sprinter and grbl.
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)

 It has preliminary support for Matthew Roberts advance algorithm
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "Marlin_main.h"

//===========================================================================
//=============================ROUTINES=============================
//===========================================================================

bool setTargetedHotend(int code);

//block all signals to imitate disabling interrupts
//inline void
//cli()
//{
//  sigprocmask(SIG_SETMASK, &global_interrupt, &old_global_interrupt);
//}
//
//inline void
//sei()
//{
//  sigprocmask(SIG_SETMASK, &old_global_interrupt, NULL);
//}

float code_value()
{
  return (strtod(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL));
}

long code_value_long()
{
  return (strtol(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL, 10));
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer, code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

void ikill()
{
  DEBUG_PRINT("kill\n");
  exit(1);
  //TODO: uninit all mraa stuffs
}

/***********************************/

int marlin_main(int argc, char *argv[]) {

  if (argc != 2) {
    printf("Wrong number of arguments provided... Provided %d instead \
            of 2\nmarlin /path/to/file\n", argc);
    exit(1);
  }

  int file = setup(argv[1]);

  while (1) loop(file);
  extern pthread_t temp_thread, stp_thread;
  if (pthread_join(temp_thread, NULL))
    errExit("pthread_join");
  if (pthread_join(stp_thread, NULL))
    errExit("pthread_join");

  return 0;
}

int setup(char *path)
{
  int file;
  struct stat s;

  if (stat(path, &s) == -1) {
    printf("Error stating %s\n", path);
    exit(1);
  }

  file_size = s.st_size;

  DEBUG_PRINT("opening file %s\n", path);
  if (!(file = open(path, O_RDONLY))) {
    perror("open file");
    exit(1);
  }

  file_buf = (char *)malloc(file_size);
  if (read(file, file_buf, file_size) < file_size) {
    printf("Error reading file\n");
    exit(1);
  }

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  DEBUG_PRINT("loading data\n");
  Config_RetrieveSettings();

  //init timer
  DEBUG_PRINT("initializing timer\n");
  if(clock_init() < 0) {
    fprintf(stderr, "Failed to init timer\n");
    exit(-1);
  }

  //init board specific data
  DEBUG_PRINT("initializing board specific data\n");
	#if MRAA == 1
  mraa_init();
  minnowmax_gpio_init();
	#endif


  //timer_init();

  //tp_init();    // Initialize temperature loop
  DEBUG_PRINT("initializing planner\n");
  plan_init();  // Initialize planner;

  //init stepper
  DEBUG_PRINT("initializing stepper\n");
  st_init();

  //init global_interrupt
  //sigfillset(&global_interrupt);
  //sigemptyset(&old_global_interrupt);

  return file;
}

void loop(int fd)
{
  if (get_command()) {
    DEBUG_PRINT("==========================================\n");
    DEBUG_PRINT("%s\n", cmdbuffer);
    process_commands();
  }

  //check heater every n milliseconds
  //manage_heater();
  manage_inactivity();
  checkHitEndstops();
}

static void run_z_probe() {
    matrix_3x3_set_to_identity(&plan_bed_level_matrix);
    feedrate = homing_feedrate[Z_AXIS];

    // move down until you find the bed
    float zPosition = -10;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

        // we have to let the planner know where we are right now as it is not where we said to go.
    zPosition = st_get_position_mm(Z_AXIS);
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS]);

    // move up the retract distance
    zPosition += home_retract_mm(Z_AXIS);
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    // move back down slowly to find bed
    feedrate = homing_feedrate[Z_AXIS]/4;
    zPosition -= home_retract_mm(Z_AXIS) * 2;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
    // make sure the planner knows where we are as it may be a bit different than we last said to move to
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}

static void clean_up_after_endstop_move() {
#ifdef ENDSTOPS_ONLY_FOR_HOMING
    enable_endstops(false);
#endif

    feedrate = saved_feedrate;
    feedmultiply = saved_feedmultiply;
    previous_millis_cmd = millis();
}

#ifdef ENABLE_AUTO_BED_LEVELING
static void set_bed_level_equation(float z_at_xLeft_yFront, float z_at_xRight_yFront, float z_at_xLeft_yBack) {
    matrix_3x3_set_to_identity(&plan_bed_level_matrix);

    vector_3 xLeftyFront;
    vector_3_init_3(&xLeftyFront, LEFT_PROBE_BED_POSITION, FRONT_PROBE_BED_POSITION, z_at_xLeft_yFront);
    vector_3 xLeftyBack;
    vector_3_init_3(&xLeftyBack, LEFT_PROBE_BED_POSITION, BACK_PROBE_BED_POSITION, z_at_xLeft_yBack);
    vector_3 xRightyFront;
    vector_3_init_3(&xRightyFront, RIGHT_PROBE_BED_POSITION, FRONT_PROBE_BED_POSITION, z_at_xRight_yFront);

    vector_3 xPositive, yPositive;
    vector_3_sub(&xRightyFront, &xLeftyFront, &xPositive);
    vector_3_get_normal(&xPositive);
    vector_3_sub(&xLeftyBack, &xLeftyFront, &yPositive);
    vector_3_get_normal(&yPositive);
    vector_3 planeNormal;
    vector_3_cross(&xPositive, &yPositive, &planeNormal);
    vector_3_get_normal(&planeNormal);

    //planeNormal.debug("planeNormal");
    //yPositive.debug("yPositive");
    plan_bed_level_matrix = matrix_3x3_create_look_at(planeNormal);
    //bedLevel.debug("bedLevel");

    //plan_bed_level_matrix.debug("bed level before");
    //vector_3 uncorrected_position = plan_get_position_mm();
    //uncorrected_position.debug("position before");

    // and set our bed level equation to do the right thing
    //plan_bed_level_matrix.debug("bed level after");

    vector_3 corrected_position = plan_get_position();
    //corrected_position.debug("position after");
    current_position[X_AXIS] = corrected_position.x;
    current_position[Y_AXIS] = corrected_position.y;
    current_position[Z_AXIS] = corrected_position.z;

    // but the bed at 0 so we don't go below it.
    current_position[Z_AXIS] = -bed_level_probe_offset[2];

    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}
#endif

static void do_blocking_move_to(float x, float y, float z) {
    float oldFeedRate = feedrate;

    feedrate = XY_TRAVEL_SPEED;

    current_position[X_AXIS] = x;
    current_position[Y_AXIS] = y;
    current_position[Z_AXIS] = z;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    feedrate = oldFeedRate;
}

static void setup_for_endstop_move() {
    saved_feedrate = feedrate;
    saved_feedmultiply = feedmultiply;
    feedmultiply = 100;
    previous_millis_cmd = millis();

    enable_endstops(true);
}

bool IsStopped() { return Stopped; };

void Stop()
{
  disable_heater();
  if(Stopped == false) {
    Stopped = true;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    ECHO_STRING(MSG_ERR_STOPPED);
    //LCD_MESSAGEPGM(MSG_STOPPED);
  }
}

//get-command() processes one line at a time
//if it is a command, return true with the command copied into cmdbuffer
//if it is an empty or comment line, return false
bool get_command()
{
  static int file_pos = 0;
  char cur_char;
  bool comment_mode = false;
  int read_count = 0;

  while (file_pos < file_size) {
    cur_char = file_buf[file_pos++];

    //if cur_char is ';', turn comment_mode on.
    //if cur_char is '\n', end of line. turn comment_mode off. Check read_count,
    ////if 0, this is an empty or comment line.
    ////if not 0, there must be something in the cmdbuffer, return.
    //if not ';' or '\n', check comment_mode.
    ////if on, do nothing.
    ////if off, copy current char into cmdbuffer and increment read_count

    if (cur_char == ';') {
      comment_mode = true;
    }
    else if (cur_char == '\n') {
      if (read_count == 0) {
        //this is an empty or comment line
        return false;
      } else {
        cmdbuffer[read_count] = '\0';
        if(strchr(cmdbuffer, 'N') != NULL) {
          errExit("line number support needed\n");
        }
        if((strchr(cmdbuffer, '*') != NULL)) {
          errExit("checksum support needed\n");
        }
        if((strchr(cmdbuffer, 'G') != NULL)){
          strchr_pointer = strchr(cmdbuffer, 'G');
          switch((int)((strtod(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL)))){
          case 0:
          case 1:
          case 2:
          case 3:
            if(Stopped == true) { // If printer is stopped by an error the G[0-3] codes are ignored.
              fprintf(stderr, "G[0-3] codes will be ignored because printer is stopped\n");
            }
            break;
          default:
            break;
          }
        }
        return true;
      }
    }
    else {
      if (!comment_mode) {
        cmdbuffer[read_count++] = cur_char;
      }
    }
  }
  return false;
}

static void axis_is_at_home(int axis) {
  current_position[axis] = base_home_pos(axis) + add_homeing[axis];
  min_pos[axis] =          base_min_pos[axis] + add_homeing[axis];
  max_pos[axis] =          base_max_pos[axis] + add_homeing[axis];
}

#define HOMEAXIS(LETTER) DEBUG_PRINT("Homing %s axis\n", #LETTER); homeaxis(LETTER##_AXIS)

static void homeaxis(int axis) {
#define HOMEAXIS_DO(LETTER) \
  ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

  if (axis==X_AXIS ? HOMEAXIS_DO(X) :
      axis==Y_AXIS ? HOMEAXIS_DO(Y) :
      axis==Z_AXIS ? HOMEAXIS_DO(Z) :
      0) {
    int axis_home_dir = home_dir(axis);

    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

    destination[axis] = 1.5 * max_length(axis) * axis_home_dir;
    feedrate = homing_feedrate[axis];
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = -home_retract_mm(axis) * axis_home_dir;
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    destination[axis] = 2*home_retract_mm(axis) * axis_home_dir;
    feedrate = homing_feedrate[axis]/2 ;
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    axis_is_at_home(axis);
    destination[axis] = current_position[axis];
    feedrate = 0.0;
    endstops_hit_on_purpose();
    axis_known_position[axis] = true;

  }
}

void do_home()
{
  int8_t i;

  DEBUG_PRINT("HOMING...\n");
#ifdef ENABLE_AUTO_BED_LEVELING
  matrix_3x3_set_to_identity(&plan_bed_level_matrix);  //Reset the plane ("erase" all leveling data)
#endif //ENABLE_AUTO_BED_LEVELING

  saved_feedrate = feedrate;
  saved_feedmultiply = feedmultiply;
  feedmultiply = 100;
  previous_millis_cmd = millis();

  enable_endstops(true);

  for(i=0; i < NUM_AXIS; i++) {
    destination[i] = current_position[i];
  }
  feedrate = 0.0;


  home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])));

  if((home_all_axis) || (code_seen(axis_codes[X_AXIS])))
  {
    HOMEAXIS(X);
  }

  if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
    HOMEAXIS(Y);
  }

  if(code_seen(axis_codes[X_AXIS]))
  {
    if(code_value_long() != 0) {
      current_position[X_AXIS]=code_value()+add_homeing[0];
    }
  }

  if(code_seen(axis_codes[Y_AXIS])) {
    if(code_value_long() != 0) {
      current_position[Y_AXIS]=code_value()+add_homeing[1];
    }
  }

  #ifndef Z_SAFE_HOMING
  if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
  #if defined (Z_RAISE_BEFORE_HOMING) && (Z_RAISE_BEFORE_HOMING > 0)
    // Set destination away from bed
    destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);
    feedrate = max_feedrate[Z_AXIS];
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS],
        destination[E_AXIS], feedrate, active_extruder);
    st_synchronize();
  #endif
    HOMEAXIS(Z);
  }
  #else                      // Z Safe mode activated.
  if(home_all_axis) {
    destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT - bed_level_probe_offset[0]);
    destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT - bed_level_probe_offset[1]);
    // Set destination away from bed
    destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);
    feedrate = XY_TRAVEL_SPEED;
    current_position[Z_AXIS] = 0;

    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],
        current_position[Z_AXIS], current_position[E_AXIS]);
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS],
        destination[E_AXIS], feedrate, active_extruder);
    st_synchronize();
    current_position[X_AXIS] = destination[X_AXIS];
    current_position[Y_AXIS] = destination[Y_AXIS];
    HOMEAXIS(Z);
  }
  // Let's see if X and Y are homed and probe is inside bed area.
  if(code_seen(axis_codes[Z_AXIS])) {
    if ( (axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]) \
      && (current_position[X_AXIS]+bed_level_probe_offset[0] >= min_pos[0]) \
      && (current_position[X_AXIS]+bed_level_probe_offset[0] <= max_pos[0]) \
      && (current_position[Y_AXIS]+bed_level_probe_offset[1] >= min_pos[1]) \
      && (current_position[Y_AXIS]+bed_level_probe_offset[1] <= max_pos[1])) {

      current_position[Z_AXIS] = 0;
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],
          current_position[Z_AXIS], current_position[E_AXIS]);
      // Set destination away from bed
      destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);
      feedrate = max_feedrate[Z_AXIS];
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS],
          destination[E_AXIS], feedrate, active_extruder);
      st_synchronize();
      HOMEAXIS(Z);
    } else if (!((axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]))) {
      fprintf(stderr, "Homeing: Position Unknown\n");
    } else {
      fprintf(stderr, "Homeing: Zprobe Out\n");
      }
    }
  #endif

  if(code_seen(axis_codes[Z_AXIS])) {
    if(code_value_long() != 0) {
      current_position[Z_AXIS]=code_value()+add_homeing[2];
    }
  }
  #ifdef ENABLE_AUTO_BED_LEVELING
  if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
    current_position[Z_AXIS] += -bed_level_probe_offset[2];  //Add Z_Probe offset (the distance is negative)
  }
  #endif
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],
      current_position[Z_AXIS], current_position[E_AXIS]);
  #ifdef ENDSTOPS_ONLY_FOR_HOMING
  enable_endstops(false);
  #endif

  feedrate = saved_feedrate;
  feedmultiply = saved_feedmultiply;
  previous_millis_cmd = millis();
  endstops_hit_on_purpose();
}

#ifdef ENABLE_AUTO_BED_LEVELING
void do_auto_bed_leveling()
{
  float x_tmp, y_tmp, z_tmp, real_z;

  #if Z_MIN_PIN == -1
  #error "You must have a Z_MIN endstop in order to enable Auto Bed Leveling feature!!! Z_MIN_PIN must point to a valid hardware pin."
  #endif

  DEBUG_PRINT("AUTO BED LEVELING...\n");

  st_synchronize();
  // make sure the bed_level_rotation_matrix is identity or the planner will get it incorectly
  //vector_3 corrected_position = plan_get_position_mm();
  //corrected_position.debug("position before G29");
  matrix_3x3_set_to_identity(&plan_bed_level_matrix);
  vector_3 uncorrected_position = plan_get_position();
  //uncorrected_position.debug("position durring G29");
  current_position[X_AXIS] = uncorrected_position.x;
  current_position[Y_AXIS] = uncorrected_position.y;
  current_position[Z_AXIS] = uncorrected_position.z;
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
  setup_for_endstop_move();

  feedrate = homing_feedrate[Z_AXIS];

  // prob 1
  do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], Z_RAISE_BEFORE_PROBING);
  do_blocking_move_to(LEFT_PROBE_BED_POSITION - bed_level_probe_offset[0], BACK_PROBE_BED_POSITION - bed_level_probe_offset[1], current_position[Z_AXIS]);

  //engage_z_probe();   // Engage Z Servo endstop if available
  run_z_probe();
  float z_at_xLeft_yBack = current_position[Z_AXIS];
  //retract_z_probe();

  ECHO_STRING("Bed x: ");
  ECHO_FLOAT(LEFT_PROBE_BED_POSITION);
  ECHO_STRING(" y: ");
  ECHO_FLOAT(BACK_PROBE_BED_POSITION);
  ECHO_STRING(" z: ");
  ECHO_FLOAT(current_position[Z_AXIS]);
  ECHO_STRING("\n");

  // prob 2
  do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);
  do_blocking_move_to(LEFT_PROBE_BED_POSITION - bed_level_probe_offset[0], FRONT_PROBE_BED_POSITION - bed_level_probe_offset[1], current_position[Z_AXIS]);

  //engage_z_probe();   // Engage Z Servo endstop if available
  run_z_probe();
  float z_at_xLeft_yFront = current_position[Z_AXIS];
  //retract_z_probe();

  ECHO_STRING("Bed x: ");
  ECHO_FLOAT(LEFT_PROBE_BED_POSITION);
  ECHO_STRING(" y: ");
  ECHO_FLOAT(FRONT_PROBE_BED_POSITION);
  ECHO_STRING(" z: ");
  ECHO_FLOAT(current_position[Z_AXIS]);
  ECHO_STRING("\n");

  // prob 3
  do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);
  // the current position will be updated by the blocking move so the head will not lower on this next call.
  do_blocking_move_to(RIGHT_PROBE_BED_POSITION - bed_level_probe_offset[0], FRONT_PROBE_BED_POSITION - bed_level_probe_offset[1], current_position[Z_AXIS]);

  //engage_z_probe();   // Engage Z Servo endstop if available
  run_z_probe();
  float z_at_xRight_yFront = current_position[Z_AXIS];
  //retract_z_probe(); // Retract Z Servo endstop if available

  ECHO_STRING("Bed x: ");
  ECHO_FLOAT(RIGHT_PROBE_BED_POSITION);
  ECHO_STRING(" y: ");
  ECHO_FLOAT(FRONT_PROBE_BED_POSITION);
  ECHO_STRING(" z: ");
  ECHO_FLOAT(current_position[Z_AXIS]);
  ECHO_STRING("\n");

  clean_up_after_endstop_move();

  set_bed_level_equation(z_at_xLeft_yFront, z_at_xRight_yFront, z_at_xLeft_yBack);

  st_synchronize();

  // The following code correct the Z height difference from z-probe position and hotend tip position.
  // The Z height on homing is measured by Z-Probe, but the probe is quite far from the hotend.
  // When the bed is uneven, this height must be corrected.
  real_z = ((float)st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS];  //get the real Z (since the auto bed leveling is already correcting the plane)
  x_tmp = current_position[X_AXIS] + bed_level_probe_offset[0];
  y_tmp = current_position[Y_AXIS] + bed_level_probe_offset[1];
  z_tmp = current_position[Z_AXIS];

  apply_rotation_xyz(plan_bed_level_matrix, &x_tmp, &y_tmp, &z_tmp);         //Apply the correction sending the probe offset
  current_position[Z_AXIS] = z_tmp - real_z + current_position[Z_AXIS];   //The difference is added to current position and sent to planner.
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}
#endif

void do_set_position()
{
  int8_t i;

  if(!code_seen(axis_codes[E_AXIS]))
    st_synchronize();
  for(i=0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i])) {
      if(i == E_AXIS) {
        current_position[i] = code_value();
        plan_set_e_position(current_position[E_AXIS]);
      } else {
        current_position[i] = code_value()+add_homeing[i];
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],
            current_position[Z_AXIS], current_position[E_AXIS]);
      }
    }
  }
}

// M109 - Wait for extruder heater to reach target.
void set_temp_and_wait()
{
  unsigned long codenum; //throw away variable

  if(setTargetedHotend(109)){
    return;
  }
  #ifdef AUTOTEMP
    autotemp_enabled=false;
  #endif
  if (code_seen('S')) {
    setTargetHotend(code_value(), tmp_extruder);
  } else if (code_seen('R')) {
    setTargetHotend(code_value(), tmp_extruder);
  }
  #ifdef AUTOTEMP
    if (code_seen('S')) autotemp_min=code_value();
    if (code_seen('B')) autotemp_max=code_value();
    if (code_seen('F'))
    {
      autotemp_factor=code_value();
      autotemp_enabled=true;
    }
  #endif

  setWatch();
  codenum = millis();

  /* See if we are heating up or cooling down */
  target_direction = isHeatingHotend(tmp_extruder); // true if heating, false if cooling

  long residencyStart;
  residencyStart = -1;
  /* continue to loop until we have reached the target temp
    _and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
  while((residencyStart == -1) || (residencyStart >= 0 &&
        (((unsigned int) (millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL))) ) {
    if( (millis() - codenum) > 1000UL )
    { //Print Temp Reading and remaining time every 1 second while heating up/cooling down
      printf("Temperature of Extruder %d: %f\n", (int)tmp_extruder, degHotend(tmp_extruder));
      if(residencyStart > -1)
      {
         codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
         printf("Remaining time to keep this temperature: %lu\n", codenum);
      }
      codenum = millis();
    }
    //manage_heater();
    manage_inactivity();
      /* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
        or when current temp falls outside the hysteresis after target temp was reached */
    if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
        (residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
        (residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS) )
    {
      residencyStart = millis();
    }
  }
  starttime=millis();
  previous_millis_cmd = millis();
}

void stop_idle_hold()
{
  if(code_seen('S')){
    stepper_inactive_time = code_value() * 1000;
  } else {
    bool all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2]))|| (code_seen(axis_codes[3])));
    if(all_axis)
    {
      st_synchronize();
      disable_e0();
      disable_e1();
      disable_e2();
      finishAndDisableSteppers();
    }
    else
    {
      st_synchronize();
      if(code_seen('X')) disable_x();
      if(code_seen('Y')) disable_y();
      if(code_seen('Z')) disable_z();
      #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
        if(code_seen('E')) {
          disable_e0();
          disable_e1();
          disable_e2();
        }
      #endif
    }
  }
}

void process_commands()
{
  int8_t i;

  if(code_seen('G')) {
    switch((int)code_value()) {
    case 0: // G0 -> G1
    case 1: // G1
      if(Stopped == false) {
        get_coordinates(); // For X Y Z E F
        prepare_move();
      }
      return;
    case 28: //G28 Home all Axis one at a time
      do_home();
      return;
    #ifdef ENABLE_AUTO_BED_LEVELING
    case 29: // G29 Detailed Z-Probe, probes the bed at 3 points.
      do_auto_bed_leveling();
      return;
    #endif
    case 90: // G90
      relative_mode = false;
      break;
    case 91: // G91
      relative_mode = true;
      break;
    case 92: // G92
      do_set_position();
     break;
    default:
      fprintf(stderr, "Unsupported G command found!\n");
      break;
    }
  } else if(code_seen('M')) {
    switch((int)code_value()) {
    case 82:
      axis_relative_modes[3] = false;
      break;
    case 83:
      axis_relative_modes[3] = true;
      break;
    case 18: //compatibility
    case 84: // M84
      stop_idle_hold();
      break;
    case 104: // M104
      if(setTargetedHotend(104)){
        break;
      }
      if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
      setWatch();
      break;
    case 106: //M106 Fan On
      if (code_seen('S')){
         fanSpeed=constrain(code_value(),0,255);
      }
      else {
        fanSpeed=255;
      }
      break;
    case 107: //M107 Fan Off
      fanSpeed = 0;
      break;
    case 109:
      set_temp_and_wait();
      break;
    #ifdef ENABLE_AUTO_BED_LEVELING
    case 212: //M212 - Set probe offset for bed leveling
    {
      for(i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          bed_level_probe_offset[i] = code_value();
        }
      }
    }
    break;
    #endif
    case 303:
    {
      float temp = 150.0;
      int e=0;
      int c=5;
      if (code_seen('E')) e=code_value();
        if (e<0)
          temp=70;
      if (code_seen('S')) temp=code_value();
      if (code_seen('C')) c=code_value();
      PID_autotune(temp, e, c);
    }
    break;
    default:
      fprintf(stderr, "Unsupported M command found!\n");
      break;
    }
  }
}

void get_coordinates()
{
  int8_t i;
  bool seen[4]={false,false,false,false};
  for(i=0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i]))
    {
      destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
      seen[i]=true;
    }
    else destination[i] = current_position[i]; //Are these else lines really needed?
  }
  if(code_seen('F')) {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
}

void clamp_to_software_endstops(float target[3])
{
  if (min_software_endstops) {
    if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
    if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
    if (target[Z_AXIS] < min_pos[Z_AXIS]) target[Z_AXIS] = min_pos[Z_AXIS];
  }

  if (max_software_endstops) {
    if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
    if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
    if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
  }
}

void prepare_move()
{
  clamp_to_software_endstops(destination);

  previous_millis_cmd = millis();

  DEBUG_PRINT("MAIN current: (%lf, %lf, %lf, %lf)\n",
      current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],
      current_position[E_AXIS]);
  DEBUG_PRINT("MAIN target: (%lf, %lf, %lf, %lf)\n",
      destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS]);
  // Do not use feedmultiply for E or Z only moves
  if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
  }
  else {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
  }
  int8_t i;
  for(i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
}

void manage_inactivity()
{
  if( (millis() - previous_millis_cmd) >  max_inactive_time )
    if(max_inactive_time)
      ikill();
  if(stepper_inactive_time)  {
    if( (millis() - previous_millis_cmd) >  stepper_inactive_time )
    {
      if(blocks_queued() == false) {
        disable_x();
        disable_y();
        disable_z();
        disable_e0();
        disable_e1();
        disable_e2();
      }
    }
  }
  check_axes_activity();
}

bool setTargetedHotend(int code){
  tmp_extruder = active_extruder;
  if(code_seen('T')) {
    tmp_extruder = code_value();
    if(tmp_extruder >= EXTRUDERS) {
      switch(code){
        case 104:
          ECHO_STRING(MSG_M104_INVALID_EXTRUDER);
          break;
        case 105:
          ECHO_STRING(MSG_M105_INVALID_EXTRUDER);
          break;
        case 109:
          ECHO_STRING(MSG_M109_INVALID_EXTRUDER);
          break;
        case 218:
          ECHO_STRING(MSG_M218_INVALID_EXTRUDER);
          break;
      }
      ECHO_DECIMAL(tmp_extruder);
      return true;
    }
  }
  return false;
}

/* vi: set et sw=2 sts=2: */
