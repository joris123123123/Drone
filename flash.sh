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

# Force a clean reset via DTR toggle so the bootloader is ready
stty -F "$PORT" 1200
sleep 0.1
stty -F "$PORT" 1200

for i in $(seq 1 5); do
    echo "  attempt $i/5..."
    if avrdude \
        -c arduino \
        -p m328p \
        -P "$PORT" \
        -b 115200 \
        -D \
        -U flash:w:$HEX:i 2>/dev/null; then
        echo "Done."
        exit 0
    fi
    sleep 1
done

echo "Flash failed after 5 attempts. Press RESET on the board and try again."
