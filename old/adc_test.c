#include <mraa.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>

#define ADC_ADDRESS 0x48

int main()
{
	int ret;
	uint8_t cmd;
	uint8_t res[2];
	uint16_t final_res;

	mraa_init();
	mraa_i2c_context i2c;

	//correct bus #?
	i2c = mraa_i2c_init_raw(0);

	if (!i2c) {
		fprintf(stderr, "init failed\n");
		exit(-1);
	}

	//address: 1001000
	ret = mraa_i2c_address(i2c, ADC_ADDRESS) ;
	if (ret != MRAA_SUCCESS) {
		fprintf(stderr, "address failed\n");
		exit(-1);
	}

	//command correct?
	//do we need to specify the register?
	for (;;) {
		cmd = 0x80;
		ret = mraa_i2c_write_byte(i2c, cmd);
		if (ret != MRAA_SUCCESS) {
			fprintf(stderr, "write failed: %d\n", ret);
			exit(-1);
		}


		//again, which register?
		ret = mraa_i2c_read(i2c, &res[0], 2);
		//if (ret != MRAA_SUCCESS) {
		//	fprintf(stderr, "read failed %d\n", ret);
		//	exit(-1);
		//}
		final_res = res[0];
		final_res = final_res << 8;
		final_res |= res[1];
		printf("read word:%u\n", final_res);
		sleep(1);
	}
}

#if 0
int main()
{
	int file, i;
  char filename[40];
	float data;
	char channel;

  sprintf(filename,"/dev/i2c-7");
  if ((file = open(filename,O_RDWR)) < 0) {
    printf("Failed to open the bus.");
    /* ERROR HANDLING; you can check errno to see what went wrong */
    exit(1);
  }
	if (ioctl(file,I2C_SLAVE,ADC_ADDRESS) < 0) {
  	printf("Failed to acquire bus access and/or talk to slave.\n");
  	/* ERROR HANDLING; you can check errno to see what went wrong */
  	exit(1);
  }

	char buf[10] = {0};
	buf[0] = 0b10000000;
  if (write(file,buf,1) != 1) {
  	/* ERROR HANDLING: i2c transaction failed */
  	printf("Failed to write to the i2c bus.\n");
  }

	for(i = 0; i<4; i++) {
  	// Using I2C Read
  	if (read(file,buf,2) != 2) {
  	  /* ERROR HANDLING: i2c transaction failed */
  	  printf("Failed to read from the i2c bus.\n");
  	} else {
  	  data = (float)((buf[0] & 0b00001111)<<8)+buf[1];
  	  data = data/4096*5;
  	  channel = ((buf[0] & 0b00110000)>>4);
  	  printf("Channel %02d Data:  %04f\n",channel,data);
  	}
  }
}
#endif
