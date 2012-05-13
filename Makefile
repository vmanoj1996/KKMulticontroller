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
	avrdude -C avrdude.conf -c usbasp-clone  -p m328p -P usb -B 8 -e -U flash:w:$<:i

#shell:
#	avrdude -c dragon_isp -p m88 -P usb -t
