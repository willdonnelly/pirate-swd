.SUFFIXES: .c .o .h
.PHONY: clean program

CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-ld
OBJCP   = arm-none-eabi-objcopy
STRIP   = arm-none-eabi-strip
CFLAGS  = -I./ -c -fno-common -Os -g -mcpu=cortex-m3 -mthumb -std=c99 \
          -D'assert_param(expr)=((void) 0)' -DSTM32F10X_LD_VL
LFLAGS  = -nostartfiles

NAME    = gpio
OBJECTS = main.o lib/STM32vldiscovery.o lib/stm32f10x_rcc.o \
          lib/stm32f10x_gpio.o lib/stm32f10x_exti.o lib/misc.o

all: $(NAME)

$(NAME): $(OBJECTS) stm32.cmd
	$(LD) -Tstm32.cmd $(LFLAGS) -o $(NAME).out $(OBJECTS)
#	$(STRIP) $(NAME).out
	$(OBJCP) -Obinary $(NAME).out $(NAME).bin
	$(OBJCP) -Oihex $(NAME).out $(NAME).hex

.c.o:
	$(CC) -c $(CFLAGS) -o $@ $<

clean:
	-rm -f $(OBJECTS) $(NAME).{out,bin}

program:
	openocd \
	-f /usr/share/openocd/scripts/interface/olimex-jtag-tiny.cfg \
	-f /usr/share/openocd/scripts/target/stm32.cfg \
	-c 'init' \
	-c 'jtag_khz 5' \
	-c 'reset halt' \
	-c 'stm32x unlock 0' \
	-c 'reset halt' \
	-c "flash write_image erase $(NAME).bin 0x08000000" \
	-c 'reset' \
	-c 'sleep 100' \
	-c 'shutdown'
