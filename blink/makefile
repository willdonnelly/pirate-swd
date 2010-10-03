.SUFFIXES: .c .o .h
.PHONY: clean

CHIP    = STM32F10X_LD_VL
NAME    = gpio
FILES   = gpio.c

CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-ld
OBJCP   = arm-none-eabi-objcopy
STRIP   = arm-none-eabi-strip
CFLAGS  = -I./ -fno-common -Os -mcpu=cortex-m3 -mthumb
LDFLAGS = -nostartfiles

OBJECTS = $(FILES:.c=.o)

all: $(NAME)

$(NAME): $(OBJECTS) stm32.cmd
	$(LD) -Tstm32.cmd $(LFLAGS) -o $(NAME).out $(OBJECTS)
	$(OBJCP) -Obinary $(NAME).out $(NAME).bin

.c.o:
	$(CC) -c $(CFLAGS) -D$(CHIP) -o $@ $<

clean:
	-rm -f $(OBJECTS) $(NAME).{out,bin}
