# ESP32-S3 Smartwatch

A fully functional smartwatch application built on the **Waveshare ESP32-S3 Touch AMOLED 2.06"** module. It combines LVGL (Light and Versatile Graphics Library) with direct display control via Arduino_GFX to deliver a rich multimedia experience on a compact AMOLED screen.

## Features

- **Digital Clock** with automatic NTP time synchronization (timezone-aware, including DST)
- **Step Counter** using the built-in QMI8658 accelerometer with peak-valley detection algorithm
- **Settings Screen** with brightness slider, battery percentage (color-coded), and WiFi status
- **File Browser** for MicroSD card with folder navigation and file type icons
- **Image Viewer** supporting JPEG, PNG, BMP, and GIF (placeholder) with auto-scaling
- **Audio Player** with play/pause, next/previous, repeat, and volume control (MP3, WAV, OGG, FLAC)
- **Text Viewer** with full vertical and horizontal scrolling (up to 500 lines, 200 chars/line)
- **Power Saving** with 3-phase auto-dimming (active > dimmed > sleep) and instant wake on touch

## Hardware

| Component | Specification |
|---|---|
| Microcontroller | ESP32-S3 (Dual-core LX7 @ 240MHz) |
| Flash | 16 MB (QIO) |
| PSRAM | 8 MB (OPI) |
| Display | CO5300 AMOLED, 410x502 pixels, 16-bit RGB565, QSPI |
| Touch | FT3168 capacitive, I2C |
| Power Management | AXP2101 PMIC (battery monitoring, charging) |
| Audio Codec | ES8311 (DAC/ADC) via I2S |
| IMU | QMI8658 (6-axis accelerometer + gyroscope) |
| Storage | MicroSD via SPI (HSPI) |
| Wireless | WiFi 802.11 b/g/n + Bluetooth 5.0 LE |

## Screenshots / Navigation

The watch has 4 main screens, navigated by horizontal swipes:

```
Clock  -->  Steps  -->  Settings  -->  Files  -->  Clock ...
```

From the file browser, tapping a file opens it in the appropriate viewer. Long press (3 seconds) returns to the browser.

## Getting Started

### Prerequisites

- [Arduino IDE 2.x](https://www.arduino.cc/en/software) (or 1.8.x)
- ESP32 Arduino Core (via Board Manager)
- USB-C data cable

### 1. Install ESP32 Board Support

In Arduino IDE, go to **File > Preferences** and add the following URL to "Additional Board Manager URLs":

```
https://espressif.github.io/arduino-esp32/package_esp32_index.json
```

Then go to **Tools > Board > Board Manager**, search for "esp32", and install **esp32 by Espressif Systems**.

### 2. Install Required Libraries

Install the following libraries via **Sketch > Include Library > Manage Libraries**:

| Library | Version | Notes |
|---|---|---|
| lvgl | 9.3.0 | UI framework |
| GFX Library for Arduino | 1.6.4 | Display driver (CO5300 QSPI) |
| XPowersLib | 0.3.2 | AXP2101 power management |
| AnimatedGIF | 2.2.0 | GIF support (placeholder) |
| PNGdec | 1.1.6 | PNG decoder |
| TJpg_Decoder | 1.1.0 | JPEG decoder |

**Manual install** (not available in Library Manager):

| Library | Version | Source |
|---|---|---|
| Arduino_DriveBus | 1.0.1 | [Waveshare SDK](https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-2.06) |
| ESP32-audioI2S | 3.4.4 | [GitHub](https://github.com/schreibfaul1/ESP32-audioI2S) |

Download these and place them in your `Arduino/libraries/` folder.

### 3. LVGL Configuration

**Critical**: Copy `lv_conf.h` from this repository to your `Arduino/libraries/` folder. It must be placed **next to** (not inside) the `lvgl/` folder:

```
Arduino/libraries/
|-- lv_conf.h          <-- HERE
|-- lvgl/
|-- GFX_Library_for_Arduino/
|-- ...
```

### 4. WiFi Configuration

Copy `wifi_config.h.example` to `wifi_config.h` and fill in your WiFi credentials:

```cpp
const char *WIFI_SSID     = "YourNetwork";
const char *WIFI_PASSWORD = "YourPassword";
const char *NTP_SERVER    = "pool.ntp.org";
const char *TZ_INFO       = "CET-1CEST,M3.5.0,M10.5.0/3";  // Brussels timezone
```

Common timezone strings:
- Netherlands/Belgium: `CET-1CEST,M3.5.0,M10.5.0/3`
- UK: `GMT0BST,M3.5.0/1,M10.5.0`
- US Eastern: `EST5EDT,M3.2.0,M11.1.0`
- US Pacific: `PST8PDT,M3.2.0,M11.1.0`

### 5. Board Settings

In **Tools** menu, configure:

| Setting | Value |
|---|---|
| Board | ESP32S3 Dev Module |
| USB CDC On Boot | **Enabled** (critical!) |
| USB DFU On Boot | Disabled |
| Flash Mode | QIO 80MHz |
| Flash Size | 16MB (128Mb) |
| Partition Scheme | app3M_fat9M_16MB |
| PSRAM | OPI PSRAM |
| CPU Frequency | 240MHz (WiFi) |
| Upload Speed | 921600 |
| USB Mode | Hardware CDC and JTAG |

### 6. Compile and Upload

1. Open `ESP32_S3_Smartwatch.ino` in Arduino IDE
2. Click **Verify** (Ctrl+R) to compile (~2.5 MB firmware)
3. Connect the watch via USB-C
4. Select the correct COM port in **Tools > Port**
5. Click **Upload** (Ctrl+U)

### 7. SD Card Setup

Format a MicroSD card as **FAT32** and create the following structure:

```
/
|-- images/
|   |-- BG_Watch_Audio.png        (REQUIRED for audio player)
|   |-- button_play_75.png        (REQUIRED for audio player)
|   |-- button_pause_75.png       (REQUIRED for audio player)
|   |-- button_prev_75.png        (REQUIRED for audio player)
|   |-- button_next_75.png        (REQUIRED for audio player)
|   |-- button_repeat_75.png      (REQUIRED for audio player)
|
|-- music/                         (your audio files)
|-- photos/                        (your images)
```

The required PNG files for the audio player are included in the `images/` folder of this repository.

## Architecture

The application uses a **dual rendering system**:

- **LVGL Mode**: Manages the 4 main screens (clock, steps, settings, files) with widget-based rendering through a PSRAM framebuffer
- **GFX Mode**: Direct pixel rendering for the file viewer (images, audio player, text) using `draw16bitRGBBitmap()` and `Arduino_Canvas` for text/overlay rendering

The CO5300 QSPI AMOLED display requires bulk pixel transfers via `draw16bitRGBBitmap()`. Individual pixel operations (`print()`, `drawLine()`, etc.) do not produce visible output. This is solved by:
- **PSRAM buffering** for PNG images (decode to buffer, then draw entire buffer at once)
- **Arduino_Canvas** (offscreen framebuffer) for text, GIF placeholder, and error screens

A boolean flag (`fileViewerActive`) switches between LVGL and GFX modes. The two systems never run simultaneously.

## Project Structure

```
ESP32_S3_Smartwatch/
|-- ESP32_S3_Smartwatch.ino    Main program (5504 lines)
|-- pin_config.h               GPIO pin definitions
|-- wifi_config.h.example      WiFi configuration template
|-- lv_conf.h                  LVGL configuration file
|
|-- ui.h / ui.cpp              LVGL UI definitions (SquareLine Studio)
|-- ui_events.h / .cpp         UI event handlers
|-- ui_helpers.h / .cpp        UI helper functions
|-- ui_Screen*.cpp             Individual screen layouts
|
|-- images/                    SD card assets (audio player buttons)
|-- documentatie/              Detailed documentation (Dutch)
```

## User Guide (Quick Reference)

| Screen | Interaction |
|---|---|
| **Clock** | Displays time, date, day name (Dutch). Auto-syncs via NTP. |
| **Steps** | Short tap: pause/resume. Long press: reset to 0. |
| **Settings** | Drag brightness slider. Battery and WiFi status shown. |
| **Files** | Tap folder to open. Tap file to view. Scroll list up/down. |
| **Image Viewer** | Swipe up/down: next/prev image. Long press: back. |
| **Audio Player** | Tap buttons: prev/play-pause/next/repeat. Swipe L/R: volume. Long press: back. |
| **Text Viewer** | Swipe up/down: scroll vertically. Swipe L/R: scroll horizontally. Long press: back. |

## Power Saving

| Phase | Trigger | Brightness |
|---|---|---|
| Active | After any touch | 200/255 |
| Dimmed | 15 sec inactivity | 50/255 |
| Sleep | 30 sec inactivity | 0/255 (screen off) |

Any touch instantly wakes the display to full brightness.

## Documentation

Detailed documentation (in Dutch) is available in the `documentatie/` folder:

1. **Project Overview** - Features, hardware, version history
2. **Hardware & Pinout** - Complete GPIO mapping and IC specifications
3. **Software Architecture** - Dual rendering system, program structure, line numbers
4. **Function Descriptions** - Detailed description of every function
5. **Libraries & Dependencies** - All required libraries and their roles
6. **Installation & Compilation** - Step-by-step build instructions
7. **User Manual** - How to operate the smartwatch

## Version History

| Version | Date | Changes |
|---|---|---|
| 2.0.0 | Jan 2026 | Initial working version with LVGL and file browser |
| 2.2.0 | Jan 2026 | Fix LVGL rendering conflict during viewer opening |
| 2.2.1 | Jan 2026 | Fix PNG endianness (BIG > LITTLE_ENDIAN) |
| 2.2.2 | Jan 2026 | Diagnostics for PNG rendering problem |
| 2.3.0 | 31 Jan 2026 | PSRAM buffer for PNG (CO5300 QSPI fix), audio player with PNG buttons, volume control |
| 2.3.1 | 1 Feb 2026 | Arduino_Canvas for text/GIF/error viewer, text viewer with vertical and horizontal scrolling |

## License

This project is provided as-is for educational and personal use.

## Author

**JWP van Renen**
