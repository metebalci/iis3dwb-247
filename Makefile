CC=gcc
CFLAGS=-I. -std=gnu11 -pthread -Wall 
# optimization options
# comment out for debugging
CFLAGS+=-O2 -march=native -fomit-frame-pointer
DEPS=
OBJS=iis3dwb-247.o
LIBS=-lpthread

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

iis3dwb-247: $(OBJS)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -rf iis3dwb-247 *.o 
