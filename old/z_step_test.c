#include <mraa.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <string.h>

timer_t timerid;
struct itimerspec its;
sigset_t mask;
mraa_gpio_context cxt;

/*
void set_time(long nanosecs)
{
  //nanosecs *= 10;
  printf("set_time: %lfus\n", nanosecs / 1000.0);
  its.it_value.tv_nsec = nanosecs;
  if (timer_settime(timerid, 0, &its, NULL) == -1)
    perror("timer_settime");
}

static void
handler(int sig, siginfo_t *si, void *uc)
{
	mraa_gpio_write(cxt, 1);
	mraa_gpio_write(cxt, 0);
	set_time(258000);
}
*/

int main()
{
	//mraa_init();
	//cxt = mraa_gpio_init_raw(475);
	//if (cxt == NULL)
	//	perror("mraa_gpio_init_raw");

	//struct sigaction sa;
  //struct sigevent sev;

  //sa.sa_flags = SA_SIGINFO;
  //sa.sa_sigaction = handler;
  //sigemptyset(&sa.sa_mask);
  //if (sigaction(SIGALRM, &sa, NULL) == -1)
	//	perror("sigaction");

  ///* block timer signal temporarily */
  //sigemptyset(&mask);
  //sigaddset(&mask, SIGALRM);
  //if (sigprocmask(SIG_SETMASK, &mask, NULL) == -1)
  //  perror("sigprocmask");

  ///* create the timer */
  //sev.sigev_notify = SIGEV_SIGNAL;
  //sev.sigev_signo = SIGALRM;
  //sev.sigev_value.sival_ptr = &timerid;
  //if (timer_create(CLOCK_REALTIME, &sev, &timerid) == -1)
  //  perror("timer_create");

  //printf("timer ID is 0x%lx\n", (long) timerid);

  ///* start the timer */
  //memset(&its, 0, sizeof(struct itimerspec));
  //set_time(500 * 0x4000);
	//sigprocmask(SIG_UNBLOCK, &mask, NULL);

	int fd; 
	while(1) {
		//mraa_gpio_write(cxt, 1);
		//mraa_gpio_write(cxt, 0);
		fd = open("/sys/class/gpio/gpio475/value", O_WRONLY);
		write(fd, "1", sizeof(char));
		close(fd);
		fd = open("/sys/class/gpio/gpio475/value", O_WRONLY);
		write(fd, "0", sizeof(char));
		close(fd);
		usleep(258);
	}
}
