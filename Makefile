# Makefile for building and testing

PROGNAME = test1
FIRMWARE ?= $(PROGNAME).bin
DEVICE ?= /dev/ttyACM0
CFLAGS := -g
CROSS ?= /usr/bin/riscv64-unknown-elf-
AS := $(CROSS)as
LD := $(CROSS)ld
OBJCOPY := $(CROSS)objcopy
OBJDUMP := $(CROSS)objdump
READELF := $(CROSS)readelf

# MCU and board specific variables
ARCH ?= rv32ec_zicsr
EMU ?= elf32lriscv

.PHONY: clean

build: $(PROGNAME).o $(PROGNAME).elf $(PROGNAME).bin $(PROGNAME).hex $(PROGNAME).dump

%.o: ./$(PROGNAME)/%.s
		$(AS) $(CFLAGS) -march=$(ARCH) -I $(PROGNAME) -o $@ $<

%.elf: %.o
		$(LD) -m $(EMU) -T linker.ld -o $@ $<

%.bin: %.elf
		$(OBJCOPY) -O binary $< $@

%.hex: %.elf
		$(OBJCOPY) -O ihex $< $@

%.dump: %.elf
		$(OBJDUMP) -D -S $< > $@

readelf: $(PROGNAME).elf
		$(READELF) -a $<

flash:
		../../installs/ch32v003fun/minichlink/minichlink -w $(FIRMWARE) 0x08000000

dumpram:
		../../installs/ch32v003fun/minichlink/minichlink -r dumpram.b 0x20000000 2048
		ghex dumpram.b

dumpprg:
		../../installs/ch32v003fun/minichlink/minichlink -r dumpprg.b 0x08000000 0x3fff
		ghex dumpprg.b

cleandump:
		rm -v *.b

clean:
		rm -v *.bin *.elf *.o *.hex *.dump
