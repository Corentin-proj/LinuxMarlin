#include <mraa.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdlib.h>

#define errExit(msg) 	do { perror(msg); exit(EXIT_FAILURE); \
										 	} while (0)

#define ADDRESS 0x48

#define TEMP_SENSOR_AD595_OFFSET 0.0
#define TEMP_SENSOR_AD595_GAIN   1.0

#define TEMP_SENSOR_0 1
#if TEMP_SENSOR_0 > 0
  #define THERMISTORHEATER_0 TEMP_SENSOR_0
  #define HEATER_0_USES_THERMISTOR
#endif

#include "thermistortables.h"

mraa_i2c_context temp_sensor;
const uint8_t cmd = 0x80;
int current_temperature_raw = 0;
float current_temperature = 0.0;

static void *heater_ttbl_map[1] = {(void *)HEATER_0_TEMPTABLE,};
static uint8_t heater_ttbllen_map[1] = {HEATER_0_TEMPTABLE_LEN,};
/*
void minnowmax_i2c_init()
{
  temp_sensor = mraa_i2c_init_raw(0);
  if (!temp_sensor) {
    errExit("mraa_i2c_init_raw");
  }

  if (mraa_i2c_address(temp_sensor, ADDRESS) != MRAA_SUCCESS)
    errExit("mraa_i2c_address");
}
*/
static float analog2temp(int raw, uint8_t e)
{
  if(heater_ttbl_map[e] != NULL)
  {
    float celsius = 0;
    uint8_t i;
    short (*tt)[][2] = (short (*)[][2])(heater_ttbl_map[e]);

    for (i=1; i<heater_ttbllen_map[e]; i++)
    {
      if ((*tt)[i][0] > raw)
      {
        celsius = (*tt)[i-1][1] +
          (raw - (*tt)[i-1][0]) *
          (float)((*tt)[i][1] - (*tt)[i-1][1]) /
          (float)((*tt)[i][0] - (*tt)[i-1][0]);
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == heater_ttbllen_map[e]) celsius = (*tt)[i-1][1];

    return celsius;
  }
  return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
}

static void updateTemperaturesFromRawValues()
{
	current_temperature = analog2temp(current_temperature_raw, 0);
	printf("current temperature is %lf\n", current_temperature);
}

/*
int main()
{
  static unsigned char temp_count = 0;
  static unsigned long raw_temp_0_value = 0;
  uint8_t res[2];
  uint16_t final_res;

	mraa_init();
	minnowmax_i2c_init();

	while(1) {
		if (mraa_i2c_write_byte(temp_sensor, cmd) != MRAA_SUCCESS)
      errExit("mraa_i2c_write_byte");
    mraa_i2c_read(temp_sensor, &res[0], 2);
    final_res = res[1];
    final_res = final_res << 8;
    final_res |= res[0];
    printf("read word: %u\n", final_res);
    raw_temp_0_value += final_res >> 2;
		temp_count++;

		if (temp_count >= 16) {
      current_temperature_raw = raw_temp_0_value;
			temp_count = 0;
			raw_temp_0_value = 0;
			updateTemperaturesFromRawValues();
		}

		sleep(1);
	}
}
*/
