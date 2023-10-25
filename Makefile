CC=gcc
CFLAGS=-I. -std=gnu11 -pthread -Wall
DEPS=
LIBS=-lpthread

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

iis3dwb: iis3dwb.o
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -rf iis3dwb *.o 
