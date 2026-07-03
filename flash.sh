#!/bin/bash

set -e

MCU=atmega328p
F_CPU=16000000UL
PORT=/dev/ttyUSB0
SRC=src/*.c
ELF=drone.elf
HEX=drone.hex

echo "[1/3] Build..."
avr-gcc \
    -mmcu=$MCU \
    -DF_CPU=$F_CPU \
    -Os \
    -Wall -Wextra -Werror \
    -o $ELF \
    $SRC

echo "[2/3] Create HEX..."
avr-objcopy -O ihex -R .eeprom $ELF $HEX

echo "[3/3] Flash..."
avrdude \
    -c arduino \
    -p m328p \
    -P "$PORT" \
    -b 115200 \
    -D \
    -U flash:w:$HEX:i

echo "Done."
