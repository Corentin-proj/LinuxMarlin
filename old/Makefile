CC=gcc
CFLAGS=-I. -lm -lrt -lmraa -lpthread -DDEBUG -ggdb
DEPS = Marlin.h


%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)


#all: Marlin_main.o Arduino.o planner.o stepper.o vector_3.o fastio.o ConfigurationStore.o temperature.o \
	timer.o
all: Marlin_main.o Arduino_marlin.o planner.o stepper.o vector_3.o fastio.o ConfigurationStore.o temperature.o
	$(CC) -o marlin Marlin_main.o Arduino_marlin.o planner.o stepper.o vector_3.o fastio.o \
		ConfigurationStore.o temperature.o $(CFLAGS)

clean:
	rm -f *.o marlin
