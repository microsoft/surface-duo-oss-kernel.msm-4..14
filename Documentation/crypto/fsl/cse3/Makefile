LINUX_INC = ../../../../include/linux/fsl
CFLAGS = -Wall -g -I$(LINUX_INC)

CC = $(CROSS_COMPILE)gcc

.PHONY: all clean

cse_test: cse_test.o

all: cse_test

clean:
	-rm -f cse_test.o cse_test
