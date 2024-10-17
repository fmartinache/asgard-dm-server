CC=gcc

CFLAGS  = -W -Wall
LDFLAGS = -lpthread -lm -lncurses
EXEC    = asgard_DM_server
LIB     = testlib.so
OBJECTS = asgard_DM_server.o ImageStreamIO.o

# adding the BMC specific things as they are setup on this machine

BMC_LIBDIR = /opt/Boston\ Micromachines/lib
BMC_INCDIR = /opt/Boston\ Micromachines/include/

CFLAGS  += -I $(BMC_INCDIR)
LDFLAGS += -L $(BMC_LIBDIR) -lBMC -lBMC_PCIeAPI -lBMC_USBAPI

# the makefile instructions

all: $(EXEC)


asgard_DM_server: $(OBJECTS)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) -o $@ -c $< $(CFLAGS)

clean:
	rm -rf *.o *.so
	rm -rf *~

mrproper: clean
	rm -rf $(EXEC)
