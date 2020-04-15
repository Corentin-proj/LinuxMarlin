#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
//#include "ultralcd.h"
#include "ConfigurationStore.h"

/*
void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size)
{
    do
    {
        eeprom_write_byte((unsigned char*)pos, *value);
        pos++;
        value++;
    }while(--size);
}
#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))
void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size)
{
    do
    {
        *value = eeprom_read_byte((unsigned char*)pos);
        pos++;
        value++;
    }while(--size);
}
#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))
//======================================================================================
*/




#define EEPROM_OFFSET 100


// IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
// in the functions below, also increment the version number. This makes sure that
// the default values are used whenever there is a change to the data, to prevent
// wrong data being written to the variables.
// ALSO:  always make sure the variables in the Store and retrieve sections are in the same order.
#define EEPROM_VERSION "V11"


#ifndef DISABLE_M503
void Config_PrintSettings()
{  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
    ECHO_STRING("Steps per unit:");
    ECHOPAIR_F("  M92 X",axis_steps_per_unit[0]);
    ECHOPAIR_F(" Y",axis_steps_per_unit[1]);
    ECHOPAIR_F(" Z",axis_steps_per_unit[2]);
    ECHOPAIR_F(" E",axis_steps_per_unit[3]);

    ECHO_STRING("Maximum feedrates (mm/s):");
    ECHOPAIR_F("  M203 X",max_feedrate[0]);
    ECHOPAIR_F(" Y",max_feedrate[1] );
    ECHOPAIR_F(" Z", max_feedrate[2] );
    ECHOPAIR_F(" E", max_feedrate[3]);
    ECHO_STRING("");

    ECHO_STRING("Maximum Acceleration (mm/s2):");
    ECHOPAIR_L("  M201 X" ,max_acceleration_units_per_sq_second[0] );
    ECHOPAIR_L(" Y" , max_acceleration_units_per_sq_second[1] );
    ECHOPAIR_L(" Z" ,max_acceleration_units_per_sq_second[2] );
    ECHOPAIR_L(" E" ,max_acceleration_units_per_sq_second[3]);
    ECHO_STRING("");
    ECHO_STRING("Acceleration: S=acceleration, T=retract acceleration");
    ECHOPAIR_F("  M204 S",acceleration );
    ECHOPAIR_F(" T" ,retract_acceleration);
    ECHO_STRING("");

    ECHO_STRING("Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
    ECHOPAIR_F("  M205 S",minimumfeedrate );
    ECHOPAIR_F(" T" ,mintravelfeedrate );
    ECHOPAIR_L(" B" ,minsegmenttime );
    ECHOPAIR_F(" X" ,max_xy_jerk );
    ECHOPAIR_F(" Z" ,max_z_jerk);
    ECHOPAIR_F(" E" ,max_e_jerk);
    ECHO_STRING("");

    ECHO_STRING("Home offset (mm):");
    ECHOPAIR_F("  M206 X",add_homeing[0] );
    ECHOPAIR_F(" Y" ,add_homeing[1] );
    ECHOPAIR_F(" Z" ,add_homeing[2] );
    ECHO_STRING("");
#ifdef PIDTEMP
    ECHO_STRING("PID settings:");
    ECHOPAIR_F("   M301 P",Kp);
    ECHOPAIR_F(" I" ,unscalePID_i(Ki));
    ECHOPAIR_F(" D" ,unscalePID_d(Kd));
    ECHO_STRING("");
#endif
    ECHO_STRING("Min position (mm):");
    ECHOPAIR_F("  M210 X" , base_min_pos[0] );
    ECHOPAIR_F(" Y" , base_min_pos[1] );
    ECHOPAIR_F(" Z" , base_min_pos[2] );
    ECHO_STRING("");
    ECHO_STRING("Max position (mm):");
    ECHOPAIR_F("  M211 X" , base_max_pos[0] );
    ECHOPAIR_F(" Y" , base_max_pos[1] );
    ECHOPAIR_F(" Z" , base_max_pos[2] );
    ECHO_STRING("");
#ifdef ENABLE_AUTO_BED_LEVELING
    ECHO_STRING("Bed probe offset (mm):");
    ECHOPAIR_F("  M212 X" , bed_level_probe_offset[0] );
    ECHOPAIR_F(" Y" , bed_level_probe_offset[1] );
    ECHOPAIR_F(" Z" , bed_level_probe_offset[2] );
    ECHO_STRING("");
#endif
}
#endif


extern void reset_acceleration_rates();
void Config_ResetDefault()
{
    float tmp1[]=DEFAULT_AXIS_STEPS_PER_UNIT;
    float tmp2[]=DEFAULT_MAX_FEEDRATE;
    long tmp3[]=DEFAULT_MAX_ACCELERATION;
		short i;
    for (i=0;i<4;i++)
    {
        axis_steps_per_unit[i]=tmp1[i];
        max_feedrate[i]=tmp2[i];
        max_acceleration_units_per_sq_second[i]=tmp3[i];
    }

    // steps per sq second need to be updated to agree with the units per sq second
    reset_acceleration_rates();

    acceleration=DEFAULT_ACCELERATION;
    retract_acceleration=DEFAULT_RETRACT_ACCELERATION;
    minimumfeedrate=DEFAULT_MINIMUMFEEDRATE;
    minsegmenttime=DEFAULT_MINSEGMENTTIME;
    mintravelfeedrate=DEFAULT_MINTRAVELFEEDRATE;
    max_xy_jerk=DEFAULT_XYJERK;
    max_z_jerk=DEFAULT_ZJERK;
    max_e_jerk=DEFAULT_EJERK;
    add_homeing[0] = add_homeing[1] = add_homeing[2] = 0;
#ifdef ENABLE_AUTO_BED_LEVELING
    bed_level_probe_offset[0] = X_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT;
    bed_level_probe_offset[1] = Y_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT;
    bed_level_probe_offset[2] = Z_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT;
#endif
#ifdef DOGLCD
    lcd_contrast = DEFAULT_LCD_CONTRAST;
#endif
#ifdef PIDTEMP
    Kp = DEFAULT_Kp;
    Ki = scalePID_i(DEFAULT_Ki);
    Kd = scalePID_d(DEFAULT_Kd);

    // call updatePID (similar to when we have processed M301)
    updatePID();

#ifdef PID_ADD_EXTRUSION_RATE
    Kc = DEFAULT_Kc;
#endif//PID_ADD_EXTRUSION_RATE
#endif//PIDTEMP

	base_min_pos[0] = X_MIN_POS_DEFAULT;
	base_min_pos[1] = Y_MIN_POS_DEFAULT;
	base_min_pos[2] = Z_MIN_POS_DEFAULT;
	base_max_pos[0] = X_MAX_POS_DEFAULT;
	base_max_pos[1] = Y_MAX_POS_DEFAULT;
	base_max_pos[2] = Z_MAX_POS_DEFAULT;

	ECHO_STRING("Hardcoded Default Settings Loaded");
}

/* vi: set et sw=2 sts=2: */
