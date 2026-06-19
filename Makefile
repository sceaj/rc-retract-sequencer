# Target configuration
MCU = atmega328p
F_CPU = 16000000
# F_CPU = 8000000 # Uncomment if using the 3.3V / 8MHz variant
PORT = /dev/cu.usbserial-AD02668J
BAUD = 57600 # The Pro Mini bootloader defaults to 57600

# Compiler options
CC = avr-gcc
CFLAGS = -Wall -Os -DF_CPU=$(F_CPU) -mmcu=$(MCU)

all: sequencer.hex

sequencer.elf: sequencer.c
	$(CC) $(CFLAGS) sequencer.c -o sequencer.elf

sequencer.hex: sequencer.elf
	avr-objcopy -O ihex -R .eeprom sequencer.elf sequencer.hex

upload: sequencer.hex
	avrdude -F -V -c arduino -p $(MCU) -P $(PORT) -b $(BAUD) -U flash:w:sequencer.hex:i

clean:
	rm -f *.elf *.hex
