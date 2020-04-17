/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include <pthread.h>
#include "Marlin.h"
#include "stepper.h"
#include "planner.h"
#include "language.h"
#include "speed_lookuptable.h"
#include <mraa.h>
#include "fastio.h"
#include "Arduino_marlin.h"


//===========================================================================
//=============================public variables  ============================
//===========================================================================
pthread_t stp_thread;

//===========================================================================
//=============================private variables ============================
//===========================================================================
//static makes it inpossible to be called from outside of this file by extern.!
static block_t *current_block;  // A pointer to the block currently being traced

//static int timerid;
//#define ENABLE_STEPPER_DRIVER_INTERRUPT()   enable_timer(timerid)
//#define DISABLE_STEPPER_DRIVER_INTERRUPT()  disable_timer(timerid)
static pthread_mutex_t stp_mtx;
static pthread_spinlock_t count_spinlock;

#define ENABLE_STEPPER_DRIVER_INTERRUPT()         \
  do {                                          \
    pthread_mutex_trylock(&stp_mtx);            \
    pthread_mutex_unlock(&stp_mtx);             \
  } while (0)

#define DISABLE_STEPPER_DRIVER_INTERRUPT() pthread_mutex_trylock(&stp_mtx)

// Variables used by The Stepper Driver Interrupt
static long acceleration_time, deceleration_time;
//static unsigned long accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
static unsigned short acc_step_rate; // needed for deccelaration start point
static char step_loops;
static unsigned short OCR1A_nominal;
static unsigned short step_loops_nominal;

volatile long endstops_trigsteps[3]={0,0,0};
static volatile bool endstop_x_hit=false, endstop_y_hit=false, endstop_z_hit=false;

//--TOM--: will be used from other threads
static bool check_endstops = true;
static volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0};

//===========================================================================
//=============================functions         ============================
//===========================================================================
#define WRITE_E_STEP(v) WRITE(E0_STEP_PIN, v)
#define NORM_E_DIR() WRITE(E0_DIR_PIN, !INVERT_E0_DIR)
#define REV_E_DIR() WRITE(E0_DIR_PIN, INVERT_E0_DIR)


#undef FORCE_INLINE
#define FORCE_INLINE static __attribute__((always_inline)) inline

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result
// XXX: to be tested!!!
#define MultiU16X8toH16(intRes, charIn1, intIn2) \
intRes = (uint16_t)(((uint32_t)charIn1 * (uint32_t)intIn2) >> 16)
/*asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %B2 \n\t" \
"movw %A0, r0 \n\t" \
"mul %A1, %A2 \n\t" \
"add %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"lsr r0 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (intRes) \
: \
"d" (charIn1), \
"d" (intIn2) \
: \
"r26" \
)*/

// intRes = longIn1 * longIn2 >> 24
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 48bit result
#define MultiU24X24toH16(intRes, longIn1, longIn2) \
intRes = (uint16_t)(((uint64_t)longIn1 * (uint64_t)longIn2) >> 24)
/*asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %B2 \n\t" \
"mov r27, r1 \n\t" \
"mul %B1, %C2 \n\t" \
"movw %A0, r0 \n\t" \
"mul %C1, %C2 \n\t" \
"add %B0, r0 \n\t" \
"mul %C1, %B2 \n\t" \
"add %A0, r0 \n\t" \
"adc %B0, r1 \n\t" \
"mul %A1, %C2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %B1, %B2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %C1, %A2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %B1, %A2 \n\t" \
"add r27, r1 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"lsr r27 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (intRes) \
: \
"d" (longIn1), \
"d" (longIn2) \
: \
"r26" , "r27" \
)*/

// Some useful constants

void checkHitEndstops()
{
 if( endstop_x_hit || endstop_y_hit || endstop_z_hit) {
   ECHO_STRING(MSG_ENDSTOPS_HIT);
   if(endstop_x_hit) {
     ECHOPAIR_F(" X:",(float)endstops_trigsteps[X_AXIS]/axis_steps_per_unit[X_AXIS]);
   }
   if(endstop_y_hit) {
     ECHOPAIR_F(" Y:",(float)endstops_trigsteps[Y_AXIS]/axis_steps_per_unit[Y_AXIS]);
   }
   if(endstop_z_hit) {
     ECHOPAIR_F(" Z:",(float)endstops_trigsteps[Z_AXIS]/axis_steps_per_unit[Z_AXIS]);
   }
   ECHO_STRING("");
   endstop_x_hit=false;
   endstop_y_hit=false;
   endstop_z_hit=false;
 }
}

void endstops_hit_on_purpose()
{
  endstop_x_hit=false;
  endstop_y_hit=false;
  endstop_z_hit=false;
}

void enable_endstops(bool check)
{
  check_endstops = check;
}

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
//
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated with the leib ramp alghorithm.

void st_wake_up() {
  //  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

void step_wait(){
  int8_t i;
   for(i=0; i < 6; i++){
   }
}


FORCE_INLINE unsigned short calc_timer(unsigned short step_rate) {
  uint32_t timer;
  if(step_rate > MAX_STEP_FREQUENCY) step_rate = MAX_STEP_FREQUENCY;

  if(step_rate > 20000) { // If steprate > 20kHz >> step 4 times
    step_rate = (step_rate >> 2)&0x3fff;
    step_loops = 4;
  }
  else if(step_rate > 10000) { // If steprate > 10kHz >> step 2 times
    step_rate = (step_rate >> 1)&0x7fff;
    step_loops = 2;
  }
  else {
    step_loops = 1;
  }

  if(step_rate < (F_CPU/500000)) step_rate = (F_CPU/500000);
  //step_rate -= (F_CPU/500000); // Correct for minimal speed
  if(step_rate >= (8*256)){ // higher step rate
    #if(F_CPU>20000000)
      // In case of high-performance processor, it is able to calculate in real-time
      timer = (uint32_t)STEPPER_TIMER_RATE / step_rate;

    #elif
      unsigned char tmp_step_rate = (step_rate & 0x00ff);
      unsigned short gain = (unsigned short)speed_lookuptable_fast[(unsigned char)(step_rate>>8)][1];
      MultiU16X8toH16(timer, tmp_step_rate, gain);
      timer = (unsigned short)speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0] - timer;

    #endif
  }
  else { // lower step rates
    #if(F_CPU>20000000)
      // In case of high-performance processor, it is able to calculate in real-time
      timer = (uint32_t)STEPPER_TIMER_RATE / step_rate;

    #elif
      timer = (unsigned short)speed_lookuptable_slow[(unsigned char)(step_rate>>3)][0];
      timer -= (((unsigned short)speed_lookuptable_slow[(unsigned char)(step_rate>>3)][1] * (unsigned char)(step_rate & 0x0007))>>3);

    #endif
  }
  if(timer < 100) { timer = 100; ECHO_STRING(MSG_STEPPER_TOO_HIGH); ECHO_DECIMAL(step_rate); }//(20kHz this should never happen)
  return timer;
}

// Initializes the trapezoid generator from the current block. Called whenever a new
// block begins.
FORCE_INLINE void trapezoid_generator_reset() {
  //struct timespec t;
  deceleration_time = 0;
  // step_rate to timer interval
  OCR1A_nominal = calc_timer(current_block->nominal_rate);
  // make a note of the number of step loops required at nominal speed
  step_loops_nominal = step_loops;
  acc_step_rate = current_block->initial_rate;
  acceleration_time = calc_timer(acc_step_rate);
  //OCR1A = acceleration_time;
  //set_time(timerid, 0, 500 * acceleration_time);
  //t.tv_sec= 0;
  //t.tv_nsec = 500 * acceleration_time;
  //nanosleep(&t, NULL);
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
//ISR(TIMER1_COMPA_vect)
//static void handler(void)
static void * handler(void * arg)
{
  DEBUG_PRINT("Stepper handler\n");
  struct timespec t;
  // Counter variables for the bresenham line tracer
  long counter_x, counter_y, counter_z, counter_e;
  unsigned char out_bits;        // The next stepping-bits to be output
  volatile unsigned long step_events_completed; // The number of step events executed in the current block
  bool old_x_min_endstop=false, old_x_max_endstop=false, old_y_min_endstop=false, old_y_max_endstop=false,
   old_z_min_endstop=false, old_z_max_endstop=false;
  signed char count_direction[NUM_AXIS] = { 1, 1, 1, 1};

  /* delay the first run (500 * 0x4000) nanoseconds */
  delayMicroseconds(500 * 0x4000 / 1000);

  while (1) {
    // If there is no current block, attempt to pop one from the buffer
    if (current_block == NULL) {
      // Anything in the buffer?
      current_block = plan_get_current_block();
      if (current_block != NULL) {
        DEBUG_PRINT("STEPPER steps to execute on each axis: (%ld, %ld, %ld, %ld)\n",
            current_block->steps_x, current_block->steps_y, current_block->steps_z,
            current_block->steps_e);
        //current_block->busy = true;
        trapezoid_generator_reset();
        counter_x = -(current_block->step_event_count >> 1);
        counter_y = counter_x;
        counter_z = counter_x;
        counter_e = counter_x;
        step_events_completed = 0;
      }
      else {
        //set_time(timerid, 0, 500 * 2000);
        t.tv_sec= 0;
        t.tv_nsec = 500 * 2000;
        //nanosleep(&t, NULL);
      }
    }

    if (current_block != NULL) {
      // Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
      out_bits = current_block->direction_bits;

      // Set the direction bits
      if((out_bits & (1<<X_AXIS))!=0){
        WRITE(X_DIR_PIN, INVERT_X_DIR);
        count_direction[X_AXIS]=-1;
      } else{
        WRITE(X_DIR_PIN, !INVERT_X_DIR);
        count_direction[X_AXIS]=1;
      }
      if((out_bits & (1<<Y_AXIS))!=0){
        WRITE(Y_DIR_PIN, INVERT_Y_DIR);
        count_direction[Y_AXIS]=-1;
      } else{
        WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
        count_direction[Y_AXIS]=1;
      }
      if ((out_bits & (1<<Z_AXIS)) != 0) {
        WRITE(Z_DIR_PIN,INVERT_Z_DIR);
        count_direction[Z_AXIS]=-1;
      } else {
        WRITE(Z_DIR_PIN,!INVERT_Z_DIR);
        count_direction[Z_AXIS]=1;
      }
      if ((out_bits & (1<<E_AXIS)) != 0) {  // -direction
        REV_E_DIR();
        count_direction[E_AXIS]=-1;
      } else { // +direction
        NORM_E_DIR();
        count_direction[E_AXIS]=1;
      }

      // check limit switches
      if (check_endstops) {
        if ((out_bits & (1<<X_AXIS)) != 0) {   // stepping along -X axis
          #if defined(X_MIN_PIN) && X_MIN_PIN > -1
            bool x_min_endstop=(READ(X_MIN_PIN) != X_MIN_ENDSTOP_INVERTING);
            if(x_min_endstop && old_x_min_endstop && (current_block->steps_x > 0)) {
              endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
              endstop_x_hit=true;
              step_events_completed = current_block->step_event_count;
              DEBUG_PRINT("X_MIN_ENDSTOP hit\n");
            }
            old_x_min_endstop = x_min_endstop;
          #endif
        }
        else { // +direction
          #if defined(X_MAX_PIN) && X_MAX_PIN > -1
            bool x_max_endstop=(READ(X_MAX_PIN) != X_MAX_ENDSTOP_INVERTING);
            if(x_max_endstop && old_x_max_endstop && (current_block->steps_x > 0)){
              endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
              endstop_x_hit=true;
              step_events_completed = current_block->step_event_count;
              DEBUG_PRINT("X_MAX_ENDSTOP hit\n");
            }
            old_x_max_endstop = x_max_endstop;
          #endif
        }

        if ((out_bits & (1<<Y_AXIS)) != 0) {   // -direction
          #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
            bool y_min_endstop=(READ(Y_MIN_PIN) != Y_MIN_ENDSTOP_INVERTING);
            if(y_min_endstop && old_y_min_endstop && (current_block->steps_y > 0)) {
              endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
              endstop_y_hit=true;
              step_events_completed = current_block->step_event_count;
              DEBUG_PRINT("Y_MIN_ENDSTOP hit\n");
            }
            old_y_min_endstop = y_min_endstop;
          #endif
        }
        else { // +direction
          #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
            bool y_max_endstop=(READ(Y_MAX_PIN) != Y_MAX_ENDSTOP_INVERTING);
            if(y_max_endstop && old_y_max_endstop && (current_block->steps_y > 0)){
              endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
              endstop_y_hit=true;
              step_events_completed = current_block->step_event_count;
              DEBUG_PRINT("Y_MAX_ENDSTOP hit\n");
            }
            old_y_max_endstop = y_max_endstop;
          #endif
        }

        if ((out_bits & (1<<Z_AXIS)) != 0) {   // -direction
          #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
            bool z_min_endstop=(READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING);
            if(z_min_endstop && old_z_min_endstop && (current_block->steps_z > 0)) {
              endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
              endstop_z_hit=true;
              step_events_completed = current_block->step_event_count;
              DEBUG_PRINT("Z_MIN_ENDSTOP hit\n");
            }
            old_z_min_endstop = z_min_endstop;
          #endif
        }
        else { // +direction
          #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
            bool z_max_endstop=(READ(Z_MAX_PIN) != Z_MAX_ENDSTOP_INVERTING);
            if(z_max_endstop && old_z_max_endstop && (current_block->steps_z > 0)) {
              endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
              endstop_z_hit=true;
              step_events_completed = current_block->step_event_count;
              DEBUG_PRINT("Z_MAX_ENDSTOP hit\n");
            }
            old_z_max_endstop = z_max_endstop;
          #endif
        }
      }

      //generate pulse to drive steppers
      int8_t i;
      for(i=0; i < step_loops; i++) { // Take multiple steps per interrupt (For high speed moves)
        counter_x += current_block->steps_x;
        pthread_spin_lock(&count_spinlock);
        if (counter_x > 0)
          count_position[X_AXIS]+=count_direction[X_AXIS];
        if (counter_y > 0)
          count_position[Y_AXIS]+=count_direction[Y_AXIS];
        if (counter_z > 0)
          count_position[Z_AXIS]+=count_direction[Z_AXIS];
        if (counter_e > 0)
          count_position[E_AXIS]+=count_direction[E_AXIS];
        pthread_spin_unlock(&count_spinlock);

        if (counter_x > 0) {
          WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
          counter_x -= current_block->step_event_count;
          //count_position[X_AXIS]+=count_direction[X_AXIS];
          WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
        }

        counter_y += current_block->steps_y;
        if (counter_y > 0) {
          WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
          counter_y -= current_block->step_event_count;
          //count_position[Y_AXIS]+=count_direction[Y_AXIS];
          WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);

        }

        counter_z += current_block->steps_z;
        if (counter_z > 0) {
          WRITE(Z_STEP_PIN, !INVERT_Z_STEP_PIN);
          counter_z -= current_block->step_event_count;
          //count_position[Z_AXIS]+=count_direction[Z_AXIS];
          WRITE(Z_STEP_PIN, INVERT_Z_STEP_PIN);

        }

        counter_e += current_block->steps_e;
        if (counter_e > 0) {
          WRITE_E_STEP(!INVERT_E_STEP_PIN);
          counter_e -= current_block->step_event_count;
          //count_position[E_AXIS]+=count_direction[E_AXIS];
          WRITE_E_STEP(INVERT_E_STEP_PIN);
        }
        step_events_completed += 1;
        if(step_events_completed >= current_block->step_event_count) break;
      }

      // Calculate new timer value
      unsigned short timer;
      unsigned short step_rate;
      if (step_events_completed <= (unsigned long int)current_block->accelerate_until) {

        MultiU24X24toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
        acc_step_rate += current_block->initial_rate;

        // upper limit
        if(acc_step_rate > current_block->nominal_rate)
          acc_step_rate = current_block->nominal_rate;

        // step_rate to timer interval
        timer = calc_timer(acc_step_rate);
        //set_time(timerid, 0, 500 * timer);
        t.tv_sec = 0;
        t.tv_nsec = 500 * timer;
        //nanosleep(&t, NULL);
        acceleration_time += timer;
      }
      else if (step_events_completed > (unsigned long int)current_block->decelerate_after) {
        MultiU24X24toH16(step_rate, deceleration_time, current_block->acceleration_rate);

        if(step_rate > acc_step_rate) { // Check step_rate stays positive
          step_rate = current_block->final_rate;
        }
        else {
          step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
        }

        // lower limit
        if(step_rate < current_block->final_rate)
          step_rate = current_block->final_rate;

        // step_rate to timer interval
        timer = calc_timer(step_rate);
        //set_time(timerid, 0, 500 * timer);
        t.tv_sec = 0;
        t.tv_nsec = 500 * timer;
        //nanosleep(&t, NULL);
        deceleration_time += timer;
      }
      else {
        //set_time(timerid, 0, 500 * OCR1A_nominal);
        t.tv_sec = 0;
        t.tv_nsec = 500 * OCR1A_nominal;
        //nanosleep(&t, NULL);
        // ensure we're running at the correct step rate, even if we just came off an acceleration
        step_loops = step_loops_nominal;
      }

     // DEBUG_PRINT("step_events_completed: %lu, step_event_count: %lu\n",
     //     step_events_completed, current_block->step_event_count);

      // If current block is finished, reset pointer
      if (step_events_completed >= current_block->step_event_count) {
        current_block = NULL;
        plan_discard_current_block();
      }
    }

    nanosleep(&t, NULL);
    //block if steppers are disabled
    pthread_mutex_lock(&stp_mtx);
    pthread_mutex_unlock(&stp_mtx);
  }
}

void st_init()
{
  //digipot_init(); //Initialize Digipot Motor Current
  //microstep_init(); //Initialize Microstepping Pins

  //Initialize Dir Pins
  SET_OUTPUT(X_DIR_PIN);
  SET_OUTPUT(Y_DIR_PIN);
  SET_OUTPUT(Z_DIR_PIN);
  SET_OUTPUT(E0_DIR_PIN);

  //Initialize Enable Pins - steppers default to disabled.
  SET_OUTPUT(X_ENABLE_PIN);
  if(!X_ENABLE_ON) WRITE(X_ENABLE_PIN,HIGH);
  SET_OUTPUT(Y_ENABLE_PIN);
  if(!Y_ENABLE_ON) WRITE(Y_ENABLE_PIN,HIGH);
  SET_OUTPUT(Z_ENABLE_PIN);
  if(!Z_ENABLE_ON) WRITE(Z_ENABLE_PIN,HIGH);
  SET_OUTPUT(E0_ENABLE_PIN);
  if(!E_ENABLE_ON) WRITE(E0_ENABLE_PIN,HIGH);

  //endstops and pullups
  #if defined(X_MIN_PIN) && X_MIN_PIN > -1
    SET_INPUT(X_MIN_PIN);
    #ifdef ENDSTOPPULLUP_XMIN
      WRITE(X_MIN_PIN,HIGH);
    #endif
  #endif

  #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
    SET_INPUT(Y_MIN_PIN);
    #ifdef ENDSTOPPULLUP_YMIN
      WRITE(Y_MIN_PIN,HIGH);
    #endif
  #endif

  #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
    SET_INPUT(Z_MIN_PIN);
    #ifdef ENDSTOPPULLUP_ZMIN
      WRITE(Z_MIN_PIN,HIGH);
    #endif
  #endif

  #if defined(X_MAX_PIN) && X_MAX_PIN > -1
    SET_INPUT(X_MAX_PIN);
    #ifdef ENDSTOPPULLUP_XMAX
      WRITE(X_MAX_PIN,HIGH);
    #endif
  #endif

  #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
    SET_INPUT(Y_MAX_PIN);
    #ifdef ENDSTOPPULLUP_YMAX
      WRITE(Y_MAX_PIN,HIGH);
    #endif
  #endif

  #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
    SET_INPUT(Z_MAX_PIN);
    #ifdef ENDSTOPPULLUP_ZMAX
      WRITE(Z_MAX_PIN,HIGH);
    #endif
  #endif

  //Initialize Step Pins
  SET_OUTPUT(X_STEP_PIN);
  WRITE(X_STEP_PIN,INVERT_X_STEP_PIN);
  disable_x();

  SET_OUTPUT(Y_STEP_PIN);
  WRITE(Y_STEP_PIN,INVERT_Y_STEP_PIN);
  disable_y();

  SET_OUTPUT(Z_STEP_PIN);
  WRITE(Z_STEP_PIN,INVERT_Z_STEP_PIN);
  disable_z();

  SET_OUTPUT(E0_STEP_PIN);
  WRITE(E0_STEP_PIN,INVERT_E_STEP_PIN);
  disable_e0();

/*
  // waveform generation = 0100 = CTC
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11);
  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0);
  TCCR1A &= ~(3<<COM1B0);

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

  OCR1A = 0x4000;
  TCNT1 = 0;
*/

  //init stepper mutex
  pthread_mutex_init(&stp_mtx, NULL);
  pthread_mutex_lock(&stp_mtx);

  //init count_position spinlock
  pthread_spin_init(&count_spinlock, PTHREAD_PROCESS_PRIVATE);

  //timerid = create_timer(handler);
  if (pthread_create(&stp_thread, NULL, &handler, NULL)) {
    errExit("pthread_create");
  }

  /* start the one-shot timer */
  //set_time(timerid, 0, 500 * 0x4000);

  ENABLE_STEPPER_DRIVER_INTERRUPT();

  enable_endstops(true); // Start with endstops active. After homing they can be disabled
  //sei();

  DEBUG_PRINT("Stepper thread initialized\n");
}

// Block until all buffered steps are executed
extern void manage_heater();
void st_synchronize()
{
  while( blocks_queued()) {
    //manage_heater();
    manage_inactivity();
    //lcd_update();
  }
}

//--TOM--: need CRITICAL_SECTION because count_position will be
//updated from stepper handler.
void st_set_position(const long x, const long y, const long z, const long e)
{
  //CRITICAL_SECTION_START;
  pthread_spin_lock(&count_spinlock);
  count_position[X_AXIS] = x;
  count_position[Y_AXIS] = y;
  count_position[Z_AXIS] = z;
  count_position[E_AXIS] = e;
  //CRITICAL_SECTION_END;
  pthread_spin_unlock(&count_spinlock);
}

void st_set_e_position(const long e)
{
  //CRITICAL_SECTION_START;
  pthread_spin_lock(&count_spinlock);
  count_position[E_AXIS] = e;
  //CRITICAL_SECTION_END;
  pthread_spin_unlock(&count_spinlock);
}

long st_get_position(uint8_t axis)
{
  long count_pos;
  //CRITICAL_SECTION_START;
  pthread_spin_lock(&count_spinlock);
  count_pos = count_position[axis];
  //CRITICAL_SECTION_END;
  pthread_spin_unlock(&count_spinlock);
  return count_pos;
}

#ifdef ENABLE_AUTO_BED_LEVELING
float st_get_position_mm(uint8_t axis)
{
  float steper_position_in_steps = st_get_position(axis);
  return steper_position_in_steps / axis_steps_per_unit[axis];
}
#endif  // ENABLE_AUTO_BED_LEVELING

void finishAndDisableSteppers()
{
  st_synchronize();
  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();
}

void quickStop()
{
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while(blocks_queued())
    plan_discard_current_block();
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

/* vi: set et sw=2 sts=2: */
