# 	
# Makefile for pulley board code.
#

CC	= g++
LIBS   := -lrt
OBJS = encbump.o 
CFLAGS = -Wall

default: $(OBJS)
	$(CC) $(OBJS) -o encoder_bump $(CFLAGS) $(LIBS)

clean:
	rm -rf *.o *~ core *.mod.* *.ko .tmp_versions .cmd
