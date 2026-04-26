# Multi-Protocol Signal Emulator

`IR_Signal_Emulator` is an STM32F407VGTx-based embedded project for a `Multi-Protocol Signal Emulator`. The firmware combines multiple short-range interfaces into one handheld system and provides menu-driven control through a `16x2` I2C LCD and a `4x4` keypad.

It supports:

- IR signal capture and replay
- Sub-GHz RF packet capture and retransmission using `CC1101`
- RFID trusted-key storage and verification using `PN532`
- NFC tag UID/NDEF read, save, clone, and basic emulation

The project is built in `STM32CubeIDE` using STM32 HAL drivers.

## Project Summary

This project is designed to capture, store, verify, clone, emulate, and retransmit signals across multiple protocols from a single embedded platform.

Supported protocol domains:

- IR communication
- RF communication
- RFID authentication
- NFC tag handling

## Main Features

### 1. IR mode

- Records IR pulse timings through timer input capture
- Stores captured IR signals in internal flash
- Replays stored IR signals using a `38 kHz` carrier
- Supports `3` flash slots: `A`, `B`, and `C`

### 2. RF mode

- Receives RF packets through the `CC1101`
- Saves captured RF payloads to flash
- Re-transmits saved packets
- Configured for `433.92 MHz`, `4.8 kbps`, `OOK`
- Supports `3` flash slots: `A`, `B`, and `C`

### 3. RFID trusted-key mode

- Scans RFID/NFC cards using the `PN532`
- Adds trusted UID entries to flash memory
- Verifies scanned keys against the trusted list
- Deletes saved trusted keys
- Supports up to `20` trusted keys
- Used as a boot authentication mechanism when the `PN532` is available

### 4. NFC mode

- Reads NFC tag UID
- Attempts to read NDEF data from MIFARE Classic sectors
- Saves UID and NDEF payload to flash
- Clones stored NDEF data to another tag
- Performs basic tag emulation through the `PN532`
- Supports `3` flash slots: `A`, `B`, and `C`

## System Overview

The STM32F407VGTx acts as the main controller and coordinates all peripherals:

- `PN532` for RFID/NFC operations
- `CC1101` for sub-GHz RF receive/transmit
- IR receiver input for pulse capture
- IR LED/transmitter output for replay
- `16x2` I2C LCD for status and menus
- `4x4` keypad for user input
- On-board LEDs for visual feedback

At startup, the firmware:

1. Initializes LCD, SPI, timers, UART, GPIO, PN532, and CC1101
2. Loads trusted RFID keys from flash
3. Checks module availability
4. If `PN532` is detected, requests a trusted key before opening the menu
5. Shows the welcome screens and enters the main menu

You can bypass boot authentication by pressing `*` during the trusted-key prompt.

## Menu Flow

### Main menu

- `1` -> IR
- `2` -> RF
- `3` -> RFID/NFC

### IR menu

- `1` -> Record IR signal
- `2` -> Transmit saved IR signal
- `D` -> Back

### RF menu

- `1` -> Record RF packet
- `2` -> Emit saved RF packet
- `D` -> Back

### RFID/NFC menu

- `1` -> RFID trusted-key functions
- `2` -> NFC functions
- `D` -> Back

### RFID menu

- `1` -> Add trusted key
- `2` -> Verify key
- `3` -> Delete trusted key
- `D` -> Back

### NFC menu

- `1` -> Record tag UID/NDEF
- `2` -> Clone saved NFC data
- `3` -> Emit/emulate saved NFC data
- `D` -> Back

## Hardware Connections

The following connections are taken from the current firmware source.

### LCD and keypad

- `I2C1`
- `PB8` -> `I2C1_SCL`
- `PB9` -> `I2C1_SDA`

Keypad matrix:

- Rows: `PC0`, `PC1`, `PC2`, `PC3`
- Columns: `PC7`, `PC6`, `PC5`, `PC4`

### SPI devices

Shared `SPI2` bus:

- `PB13` -> `SPI2_SCK`
- `PB14` -> `SPI2_MISO`
- `PB15` -> `SPI2_MOSI`

`PN532` chip select:

- `PB12` -> `PN532_CS`

`CC1101` chip select and status pins:

- `PB11` -> `CC1101_CS`
- `PA8` -> `CC1101_GDO0`

Note: the firmware switches SPI bit order at runtime because the `CC1101` uses MSB-first while the current `PN532` SPI implementation is configured for LSB-first on the shared bus.

### IR interface

- `PA1` -> IR input capture
- `PA6` -> `TIM3_CH1` PWM output for IR transmit

Timer usage:

- `TIM2` -> IR pulse capture
- `TIM3` -> `38 kHz` carrier generation

### UART

- `USART2` on `PA2/PA3` at `115200`
- `USART3` on `PD8/PD9` at `115200`

### User button and LEDs

- `PA0` -> user button
- `PD12`, `PD13`, `PD14`, `PD15` -> status LEDs

## Flash Storage Layout

The project stores captured data in STM32 internal flash.

### IR storage

- Base address: `0x08060000`
- Sector: `FLASH_SECTOR_7`
- Slots: `A`, `B`, `C`
- Slot size: `4096` bytes
- Data: pulse count and pulse timing array

### NFC storage

- Base address: `0x08040000`
- Sector: `FLASH_SECTOR_6`
- Slots: `A`, `B`, `C`
- Slot size: `1024` bytes
- Data: UID length, UID bytes, NDEF length, NDEF payload

### RF storage

- Base address: `0x08020000`
- Sector: `FLASH_SECTOR_5`
- Slots: `A`, `B`, `C`
- Data: payload length and RF packet bytes

### Trusted RFID key storage

- Base address: `0x08010000`
- Sector: `FLASH_SECTOR_4`
- Capacity: `20` trusted keys
- Data: UID length and UID bytes for each saved key

## Source Structure

- [Core/Src/main.c](C:/Users/rajes/STM32CubeIDE/workspace_1.19.0/IR_Signal_Emulator/Core/Src/main.c)  
  Main application logic, menu state machine, flash storage, IR/RF/RFID/NFC workflows
- [Core/Src/cc1101.c](C:/Users/rajes/STM32CubeIDE/workspace_1.19.0/IR_Signal_Emulator/Core/Src/cc1101.c)  
  CC1101 driver and RF packet handling
- [Core/Src/pn532_spi.c](C:/Users/rajes/STM32CubeIDE/workspace_1.19.0/IR_Signal_Emulator/Core/Src/pn532_spi.c)  
  PN532 SPI driver, UID read, NDEF read/write, tag emulation helpers
- [Core/Src/lcd_i2c.c](C:/Users/rajes/STM32CubeIDE/workspace_1.19.0/IR_Signal_Emulator/Core/Src/lcd_i2c.c)  
  LCD interface functions
- [Core/Src/keypad.c](C:/Users/rajes/STM32CubeIDE/workspace_1.19.0/IR_Signal_Emulator/Core/Src/keypad.c)  
  Keypad scanning logic
- [IR_Signal_Emulator.ioc](C:/Users/rajes/STM32CubeIDE/workspace_1.19.0/IR_Signal_Emulator/IR_Signal_Emulator.ioc)  
  STM32CubeMX configuration

## Build and Run

### Requirements

- `STM32CubeIDE`
- STM32F407VGTx target board or equivalent hardware
- Connected peripherals:
  - `PN532`
  - `CC1101`
  - IR receiver
  - IR transmitter LED/module
  - `16x2` I2C LCD
  - `4x4` keypad

### Steps

1. Open `STM32CubeIDE`
2. Import the project from this folder if it is not already in the workspace
3. Open [IR_Signal_Emulator.ioc](C:/Users/rajes/STM32CubeIDE/workspace_1.19.0/IR_Signal_Emulator/IR_Signal_Emulator.ioc) if you want to review or regenerate CubeMX settings
4. Build the project
5. Flash the firmware to the STM32F407 target
6. Reset the board and interact using the LCD and keypad

## How It Works

### IR capture and replay

- IR input edges are captured using `TIM2` input capture interrupts
- Pulse widths are stored in the `ir_pulses[]` array
- Saved pulse sequences are written to flash
- Replay is performed by enabling/disabling the `38 kHz` PWM carrier on `TIM3`

### RF capture and replay

- The `CC1101` is placed in RX mode and waits for a packet
- Received payload bytes are stored in RAM, then optionally saved to flash
- Saved payloads can later be retransmitted using the same module settings

### RFID trusted keys

- Card UIDs are read through the `PN532`
- Trusted UIDs are stored in flash
- During boot, scanned UIDs are matched against the trusted list
- A matching key grants access to the main menu

### NFC record, clone, and emulate

- The firmware reads the UID first
- It then attempts to read NDEF content from a MIFARE Classic tag
- UID and NDEF data can be saved into one of three flash slots
- Saved NDEF data can be written to another compatible tag
- Basic emulation repeatedly calls the PN532 tag emulation helper until stopped

## Current Limitations

- IR storage supports up to `200` pulse entries per captured signal
- RF payload size is limited to `61` bytes
- NFC NDEF handling is focused on simple MIFARE Classic access
- NFC emulation is basic and may not behave like a complete commercial tag
- Flash sectors are erased when updating stored slots, so the firmware rewrites all slots in that sector each time
- The project currently has no PC-side configuration utility

## Notes

- `Debug/` contains generated build outputs and can be regenerated by the IDE
- Some helper functions and drivers are tightly coupled to the current hardware wiring
- If you change pins or peripherals in CubeMX, review the keypad, PN532, CC1101, and timer-related code carefully

## Future Improvements

- Add a proper wiring diagram or block diagram
- Add support for more RF protocols and configuration profiles
- Improve NFC compatibility and tag emulation behavior
- Add export/import support for saved IR, RF, and NFC data
- Add serial debug logs for easier testing

## License

This project includes STM32 HAL and CMSIS files from STMicroelectronics under their respective license terms. Add your project-specific license here if you plan to publish the repository.
