# Name: Makefile
# Author: <insert your name here>
# Copyright: <insert your copyright message here>
# License: <insert your license reference here>

# This is a prototype Makefile. Modify it according to your needs.
# You should at least check the settings for
# DEVICE ....... The AVR device you compile for
# CLOCK ........ Target AVR clock rate in Hertz
# OBJECTS ...... The object files created from your source files. This list is
#                usually the same as the list of source files with suffix ".o".
# PROGRAMMER ... Options to avrdude which define the hardware you use for
#                uploading to the AVR and the interface where this hardware
#                is connected.
# FUSES ........ Parameters for avrdude to flash the fuses appropriately.

DEVICE     = atmega88
CLOCK      = 20000000
PROGRAMMER = -c usbtiny
OBJECTS    = main.o spi.o mcp23s17.o auart.o
#atmega88 version
FUSES	   = -U hfuse:w:0xdd:m -U lfuse:w:0xfe:m -U efuse:w:0x01:m
#final version - 20 MHz crystal
#FUSES      = -U hfuse:w:0xdd:m -U lfuse:w:0xfe:m -U efuse:w:0x01:m
#debug version - 16 MHz crystal, 2 MHz effective. Perfect for those with shitty test equipment.
#FUSES      = -U hfuse:w:0xdd:m -U lfuse:w:0x7e:m -U efuse:w:0x01:m

# Tune the lines below only if you know what you are doing:

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE) -B 10 -F
COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)

# symbolic targets:
all:	servomatrix.hex	size

spi.o:			spi.c		spi.h
mcp23s17.o:		spi.o		mcp23s17.h		mcp23s17.c
main.o:			spi.o		mcp23s17.o		auart.o
auart.o:		auart.c		auart.h

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:servomatrix.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

size:	servomatrix.elf
	avr-size --mcu=$(DEVICE) --format=avr servomatrix.elf
	
ident:
	$(AVRDUDE)

# Xcode uses the Makefile targets "", "clean" and "install"
install: flash fuse

# if you use a bootloader, change the command below appropriately:
load: all
	bootloadHID servomatrix.hex

clean:
	rm -f servomatrix.hex servomatrix.elf $(OBJECTS)

# file targets:
servomatrix.elf: $(OBJECTS)
	$(COMPILE) -o servomatrix.elf $(OBJECTS)

servomatrix.hex: servomatrix.elf
	rm -f servomatrix.hex
	avr-objcopy -j .text -j .data -O ihex servomatrix.elf servomatrix.hex
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	servomatrix.elf
	avr-objdump -d servomatrix.elf

cpp:
	$(COMPILE) -E main.c
