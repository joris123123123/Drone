#!/bin/bash

set -e

MCU=atmega328p
F_CPU=16000000UL
PORT=/dev/ttyUSB0

echo "[1/3] Build..."

avr-gcc \
  -mmcu=$MCU \
  -DF_CPU=$F_CPU \
  -Os \
  -Wall \
  -Wextra \
  -o drone.elf \
  src/*.c

echo "[2/3] Create HEX..."

avr-objcopy \
  -O ihex \
  -R .eeprom \
  drone.elf \
  drone.hex

echo "[3/3] Flash..."

avrdude \
  -c arduino \
  -p m328p \
  -P "$PORT" \
  -b 115200 \
  -D \
  -U flash:w:drone.hex:i

echo "Done."
