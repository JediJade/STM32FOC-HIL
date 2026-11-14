CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size

# UnityMbed passes PROJECT=<project name>; a plain `make` builds build/firmware.*
PROJECT ?= firmware
TARGET   = $(PROJECT)
OUTDIR   = build
CPU      = cortex-m3

#   src/startup.c     vector table + Reset_Handler   (the drive's, unchanged)
#   src/usb_cdc.c     USB CDC device on the registers (the drive's, unchanged)
#   src/vmotor.c      the motor model                 (the drive's, unchanged)
#   src/foc.c         Clarke and the sine table       (the drive's, unchanged)
#   src/main.c        clock, the 20 kHz model tick, the console loop
#   src/pwmin.c       the three phase duties, measured off the drive's pins
#   src/sensors.c     the Hall pins the drive reads back
SRCS := $(wildcard src/*.c)

# The same chip as the drive, at the same clock, with the same flags. That
# is the point of using a second Blue Pill: the model steps in a period of
# exactly the same length, and the arithmetic is bit for bit what the drive
# would have done with it.
CFLAGS  = -mcpu=$(CPU) -mthumb -Os -g -ffreestanding -Wall -Wextra \
          -Iinc -Icmsis -DSTM32F103xB $(CFLAGS_EXTRA)
LDFLAGS = -nostdlib -Wl,-T,linker.ld -Wl,--gc-sections -Wl,-Map=$(OUTDIR)/$(TARGET).map

all: $(OUTDIR)/$(TARGET).elf $(OUTDIR)/$(TARGET).bin $(OUTDIR)/$(TARGET).hex

$(OUTDIR)/$(TARGET).elf: $(SRCS) linker.ld
	@mkdir -p $(OUTDIR)
	$(CC) $(CFLAGS) $(SRCS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(OUTDIR)/$(TARGET).bin: $(OUTDIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(OUTDIR)/$(TARGET).hex: $(OUTDIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

flash: $(OUTDIR)/$(TARGET).hex
	openocd -f openocd.cfg -c "program $< verify reset exit"

clean:
	rm -rf $(OUTDIR)

.PHONY: all clean flash
