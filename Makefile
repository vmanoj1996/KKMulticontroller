all: build

build: kk.hex

kk.raw: kk.o
	avr-objcopy -j .text -j .data -O binary $< $@

kk.hex: kk.o
	avr-objcopy -j .text -j .data -O ihex $< $@

kk.o: kk.c io_cfg.h typedefs.h
	avr-gcc -mmcu=atmega328p -Wall -g3 -gdwarf-2 -std=gnu99 -DF_CPU=8000000 -Os -fsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -o $@ $<
#	size $@

kk.s: kk.c io_cfg.h typedefs.h
	avr-gcc -mmcu=atmega328p -Wall -g3 -gdwarf-2 -std=gnu99 -DF_CPU=8000000 -Os -fsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -S -o $@ $<

program: kk.hex
	avrdude.exe -C avrdude.conf -p m328p -P usb -c usbasp-clone -B 8 -e -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m
	avrdude.exe -C avrdude.conf -p m328p -P usb -c usbasp-clone -B 8 -e -U flash:w:$<:i 
