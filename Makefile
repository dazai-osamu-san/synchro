MCU=atmega328p
F_CPU=8000000UL
BAUD=57600
PROGRAMMER=arduino
PORT=/dev/ttyUSB0

CC=avr-gcc
OBJCOPY=avr-objcopy
AVRDUDE=avrdude

CFLAGS=-mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Iinclude
LDFLAGS=-mmcu=$(MCU)

TARGET=blinky
SRC=src/main.c
BUILD_DIR=build

all: $(BUILD_DIR)/$(TARGET).hex

$(BUILD_DIR)/$(TARGET).elf: $(SRC)
	mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $^

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

flash: $(BUILD_DIR)/$(TARGET).hex
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -P $(PORT) -b $(BAUD) -U flash:w:$<:i

clean:
	rm -rf $(BUILD_DIR)
