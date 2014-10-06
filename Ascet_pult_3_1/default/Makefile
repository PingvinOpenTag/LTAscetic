###############################################################################
# Makefile for the project LTAscetic_Atmega32
###############################################################################

## General Flags
PROJECT = LTAscetic_Atmega32
MCU = atmega32
TARGET = LTAscetic_Atmega32.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99                                             -DF_CPU=16000000UL -O1 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=LTAscetic_Atmega32.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Objects that must be built in order to link
OBJECTS = ltag_ascetic.o global_variables.o isr.o joystick_driver.o usart.o menu.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) LTAscetic_Atmega32.hex LTAscetic_Atmega32.eep LTAscetic_Atmega32.lss size

## Compile
ltag_ascetic.o: ../ltag_ascetic.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

global_variables.o: ../global_variables.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

isr.o: ../isr.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

joystick_driver.o: ../joystick_driver.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

usart.o: ../usart.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

menu.o: ../menu.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) LTAscetic_Atmega32.elf dep/* LTAscetic_Atmega32.hex LTAscetic_Atmega32.eep LTAscetic_Atmega32.lss LTAscetic_Atmega32.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

