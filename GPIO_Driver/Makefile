#Makefile for automated compilation of file

GCC = arm-none-eabi-gcc #specify C++ compiler

CPPFLAGS = -c -std=gnu11 -Ofast -finline-functions -mcpu=cortex-m4 -mthumb -Iinc/ -I/home/chabeli/Desktop/Repositories/STM32F4xx_Drivers/inc/

LDFLAGS =  -T linker/stm32f4_linker.ld -Wl,-Map=final.map -mcpu=cortex-m4 -mthumb -nostdlib


all: bin/final.elf obj/main.o  obj/stm32f4_start_up.o 


obj/main.o: src/main.c
	$(GCC) $(CPPFLAGS) -o $@ $^

obj/stm32f4_start_up.o: ~/Desktop/Repositories/STM32F4xx_Drivers/src/stm32f4_start_up.c
	$(GCC) $(CPPFLAGS) -o $@ $^

bin/final.elf: obj/main.o obj/stm32f4_start_up.o 
	$(GCC) $(LDFLAGS) -o $@ $^

clean:
	rm bin/final.elf obj/*.o *.map

load:
	openocd -f board/st_nucleo_f4.cfg

debug:
	arm-none-eabi-gdb

elf: bin/final.elf
	arm-none-eabi-objdump -D bin/final.elf