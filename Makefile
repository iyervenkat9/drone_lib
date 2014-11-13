COMPILE_FOR_ARM = yes
APP_NAME = serial_test
SRC = actuation.c convert.c navdata.c serial.c

APP_SRC = $(APP_NAME).c

OBJ = $(SRC:.c=.o)
 
OUT = libdrone.a

EXE_BIN = $(APP_NAME)
 
# include directories
INCLUDES = -I. 
 
# compiler
ifeq ($(COMPILE_FOR_ARM), yes)
CCC = arm-linux-gnueabi-gcc
CFLAGS+=-DON_PC=0
else
CCC = gcc
CFLAGS+=-DON_PC=1
endif 
# library paths
LIBS = -lm -lpthread
 
.SUFFIXES: .c
 
default: $(OUT) $(EXE_BIN)

clean:
	rm libdrone.a
	rm convert.o
	rm actuation.o
	rm navdata.o
	rm serial.o
	rm $(APP_NAME)	
 
.c.o:
	$(CCC) $(CFLAGS) $(INCLUDES) -c $< -o $@
 
$(OUT): $(OBJ)
	
	ar -cvq $(OUT) $(OBJ)

$(EXE_BIN): 
	$(CCC) $(CFLAGS) $(INCLUDES) $(APP_SRC) -L. -ldrone -lpthread -lm $< -o $@
 
 
