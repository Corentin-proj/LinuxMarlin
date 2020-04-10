#include <fcntl.h>
#include <unistd.h>
#include <limits.h>
#include <sys/sysinfo.h>
#include <sys/time.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "Marlin.h"
#include "fastio.h"

//static uint64_t tsc_init = 0;
static struct timeval start;
static float clocks_per_ns = 0;
float cpufreq = 0;

static inline uint64_t rdtsc(void)
{
    uint32_t lo, hi;
    uint64_t returnVal;
    /* We cannot use "=A", since this would use %rax on x86_64 */
    __asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
    returnVal = hi;
    returnVal <<= 32;
    returnVal |= lo;

    return returnVal;
}

unsigned long millis( void )
{
    /* similar to the micros() function, it returns ms since sketch start up time.
     The underlying counter is a 64 bit value, but the representation of millis
     as unsigned 32-bits means it recycles in ~ 1190 hours.*/

    //uint64_t tsc_cur = rdtsc(), diff = 0, divisor = 0;
    //divisor = (cpufreq * 1000);
    //diff = tsc_cur - tsc_init;
    struct timeval end;
    uint64_t diff;

    gettimeofday(&end, NULL);
    diff = (end.tv_sec * 1000000 + end.tv_usec)
            - (start.tv_sec * 1000000 + start.tv_usec);

    return (diff / 1000);
}

/* TSC snapshot */
int clock_init(void)
{
    int cpufreq_fd, ret;
    char buf[0x400];
    char * str = 0, * str2 = 0;
    char * mhz_str = "cpu MHz\t\t: ";

    /* Grab initial TSC snapshot */
    //tsc_init = rdtsc();
    gettimeofday(&start, NULL);

    cpufreq_fd = open("/proc/cpuinfo", O_RDONLY);
    if( cpufreq_fd < 0){
        fprintf(stderr, "unable to open /proc/cpuinfo\n");
        return -1;
    }
    memset(buf, 0x00, sizeof(buf));
    ret = read(cpufreq_fd, buf, sizeof(buf));
    if ( ret < 0 ){
        fprintf(stderr, "unable to read cpuinfo !\n");
        close(cpufreq_fd);
        return -1;
    }
    close(cpufreq_fd);
    str = strstr(buf, mhz_str);
    if (!str){
        fprintf(stderr, "Buffer %s does not contain CPU frequency info !\n", buf);
        return -1;
    }

    str += strlen(mhz_str);
    str2 = str;

    while(str2 < buf  + sizeof(buf)-1 && *str2 != '\n'){
        str2++;
    }
    if(str2 == buf + sizeof(buf-1) && *str2 !='\n'){
        fprintf(stderr, "malformed cpufreq string %s\n", str);
        return -1;
    }
    *str2 = '\0';
    cpufreq = atof(str);


    printf("cpufrequency is %f mhz\n", cpufreq);

    /* Calculate nanoseconds per clock */
    clocks_per_ns = 1000/cpufreq;

    printf("nanoseconds per clock %f\n", clocks_per_ns);
}

int digitalRead(int pin)
{
  return READ(pin);
}

void digitalWrite(int pin, int val)
{
  WRITE(pin, val);
}

//void delay(unsigned long time)
//{
//  unsigned long start = millis();
//  while (millis() - start < time) ;
//}

void delay(unsigned long ms)
{
  usleep(ms * 1000);
}

void delayMicroseconds(unsigned long us)
{
	usleep(us);
}

float constrain(float x, float a, float b)
{
  if (x >= a && x <= b) return x;
  else if (x < a) return a;
  else return b;
}

/* vi: set et sw=2 sts=2: */
