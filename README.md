# Grid synchronisation

This repository contains an easy-to-implement grid synchronisation circuit.

## Content
- **Hardware**: KiCad schematic and layout design for isolated grid frequency acquisition.
- **Software**: Implementation for an STM32 Nucleo H755 for grid synchronisation.

## Functionality
The circuit uses opto-emulators (ISOM8710) for isolated detection of the mains zero crossing. A microcontroller analyses the signal edges to calculate the mains frequency and phase position.