MCU=atmega8
CC=avr-gcc
#file=Malingping
file=optimalkan
#file=spektrum
#file=kosongan
#file=utama-24juni2009
# main
OBJCOPY=avr-objcopy
pengukur=avr-size
# optimize for size:
CFLAGS=-g -mmcu=$(MCU) -Wall -Wstrict-prototypes -Os -mcall-prologues

all: clean avrledtest.hex show_size
avrledtest.hex : avrledtest.out 
	$(OBJCOPY) -R .eeprom -O ihex $(file).out $(file).hex 
avrledtest.out : avrledtest.o 
	$(CC) $(CFLAGS) -o $(file).out -Wl,-Map,$(file).map $(file).o 
#avrledtest.o : avrledtest.c 
avrledtest.o : $(file).c 
	$(CC) $(CFLAGS) -Os -c $(file).c

# erase the AVR before it is programmed
tulis:
	avrdude -F -c usbasp -p m8 -U flash:w:$(file).hex
#	uisp -dlpt=/dev/parport0 --erase  -dprog=dapa
#	uisp -dlpt=/dev/parport0 --upload if=avrledtest.hex -dprog=dapa  -v=3 --hash=32

hapus:
	rm -f *.o *.map *.out *.hex

clean: hapus

show_size:
	$(pengukur) $(file).o
