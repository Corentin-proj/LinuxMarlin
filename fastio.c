#include "Marlin.h"
#include "fastio.h"
#include <string.h>

struct gpio_context gpio_cxt[NGPIO];

static const int minnowmax_pin_mapping[NGPIO+1] = {
	-1, -1, -1, -1, -1, 476, 481,
	477, 480, 478, 483, 479, 482,
	499, 472, 498, 473, 485, 475,
	484, 474, 338, 504, 339, 505,
	340, 464, 
};

static const char minnowmax_pin_assignment[NGPIO+1][10] = {
	"NONEXIST", "GND", "GND", "+5V", "+3V",
	"HEATER", "X_STEP", "SPI_MISO", "X_DIR",
	"SPI_MOSI", "X_ENABLE", "Z_ENABLE", "Y_STEP",
	"I2C_SCL", "Y_DIR", "I2C_SDA", "Y_ENABLE",
	"E0_STEP", "Z_STEP", "E0_DIR", "Z_DIR",
	"X_STOP", "UNUSED", "Y_STOP", "FAN",
	"Z_STOP", "E0_ENABLE",
};

void minnowmax_gpio_init()
{
	int i;
	for (i = 0; i < NGPIO+1; i++) {
		//set linux mapping
		gpio_cxt[i].linux_mapping = minnowmax_pin_mapping[i];
		//set pin name
		strcpy(gpio_cxt[i].pin_name, minnowmax_pin_assignment[i]);
	}
}

void SET_OUTPUT(unsigned IO)
{
        DEBUG_PRINT("set output %d: %d\n", IO, GET_OS_MAPPING(IO));
	DEBUG_PRINT("setting up pin %s\n", gpio_cxt[IO].pin_name);
        if (IO > NGPIO) return;
	if (!gpio_cxt[IO].mraa_cxt) {
		gpio_cxt[IO].mraa_cxt = mraa_gpio_init_raw(GET_OS_MAPPING(IO));
		if (!gpio_cxt[IO].mraa_cxt) {
			errExit("mraa_gpio_init_raw");
		}
	}
	mraa_gpio_dir(gpio_cxt[IO].mraa_cxt, MRAA_GPIO_OUT);
}

void SET_INPUT(unsigned IO)
{
	//DEBUG_PRINT("setting up pin %s\n", gpio_cxt[IO].pin_name);
        if (IO > NGPIO) return;
	if (!gpio_cxt[IO].mraa_cxt) {
		gpio_cxt[IO].mraa_cxt = mraa_gpio_init_raw(GET_OS_MAPPING(IO));
		if (!gpio_cxt[IO].mraa_cxt) {
			errExit("mraa_gpio_init_raw");
		}
	}
	mraa_gpio_dir(gpio_cxt[IO].mraa_cxt, MRAA_GPIO_IN);
}

void WRITE(unsigned IO, int v)
{
	//DEBUG_PRINT("writing to pin %s\n", gpio_cxt[IO].pin_name);
        if (IO > NGPIO) return;
	if (!gpio_cxt[IO].mraa_cxt) {
		errExit("write to uninitialized gpio");
	}
	mraa_gpio_write(gpio_cxt[IO].mraa_cxt, v);
}

int READ(unsigned IO)
{
        if (IO > NGPIO) 
          errExit("invalid pin\n");
	if (!gpio_cxt[IO].mraa_cxt) {
		errExit("read from uninitialized gpio");
	}
	return mraa_gpio_read(gpio_cxt[IO].mraa_cxt);
}

/* vi: set et sw=2 sts=2: */
