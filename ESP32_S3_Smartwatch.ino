/*
 * ====================================================================================
 * BESTANDSNAAM: ESP32_S3_Smartwatch.ino
 * ====================================================================================
 *
 * VERSIE:       2.3.1
 * DATUM:        31 januari 2026, 23:45 uur (Tijdzone: Europa/Brussel)
 * AUTEUR:       JWP van Renen
 *
 * ====================================================================================
 * BESCHRIJVING:
 * ====================================================================================
 *
 * Dit programma implementeert een smartwatch applicatie voor de Waveshare ESP32-S3
 * Touch AMOLED 2.06 inch display module. De applicatie maakt gebruik van LVGL
 * (Light and Versatile Graphics Library) versie 9.3.0 voor de grafische interface.
 *
 * De smartwatch biedt een complete gebruikerservaring met meerdere schermen,
 * touch-navigatie, en automatische energiebesparing om de batterijduur te maximaliseren.
 *
 * ====================================================================================
 * HARDWARE COMPONENTEN:
 * ====================================================================================
 *
 * 1. MICROCONTROLLER: ESP32-S3
 *    - Dual-core Xtensa LX7 processor @ 240MHz
 *    - Ingebouwde WiFi 802.11 b/g/n (2.4GHz)
 *    - Ingebouwde Bluetooth 5.0 LE
 *    - 8MB PSRAM voor extra werkgeheugen
 *    - 16MB Flash voor programmaopslag
 *
 * 2. DISPLAY: CO5300 AMOLED
 *    - Resolutie: 410 x 502 pixels
 *    - Kleurdiepte: 16-bit RGB565 (65.536 kleuren)
 *    - Interface: QSPI (Quad SPI) voor hoge snelheid
 *    - Helderheidsregeling: 0-255 niveaus via MIPI commando
 *
 * 3. TOUCH CONTROLLER: FT3168
 *    - Type: Capacitief multi-touch
 *    - Interface: I2C @ adres 0x38
 *    - Interrupt-driven voor lage latentie
 *    - Ondersteunt gestures (swipe, tap, long press)
 *
 * 4. POWER MANAGEMENT: AXP2101
 *    - Batterij laden (Li-Ion/Li-Po)
 *    - Voltage regulatie voor alle componenten
 *    - Batterij niveau monitoring (percentage)
 *    - Laadstatus detectie
 *
 * ====================================================================================
 * FUNCTIONALITEITEN:
 * ====================================================================================
 *
 * SCHERM 1 - KLOKSCHERM (ui_ScreenClock):
 *    - Grote digitale tijdweergave (uren:minuten)
 *    - Aparte secondenweergave
 *    - Datum in formaat: "dag maand jaar" (bijv. "17 Januari 2026")
 *    - Dagnaam in het Nederlands (bijv. "Vrijdag")
 *    - Tijd wordt automatisch gesynchroniseerd via NTP (Network Time Protocol)
 *    - Tijdzone: Europa/Brussel (CET in winter, CEST in zomer)
 *
 * SCHERM 2 - STAPPENSCHERM (ui_ScreenSteps):
 *    - Stappenteller weergave
 *    - Tik om tellen te pauzeren/hervatten
 *    - Lang drukken (>1 seconde) om te resetten naar 0
 *    - Visuele feedback bij interactie
 *
 * SCHERM 3 - INSTELLINGENSCHERM (ui_ScreenSettings):
 *    - Helderheidsregeling via slider (0-255)
 *    - Batterijpercentage met kleurcodering:
 *      * Groen (>50%): Goed geladen
 *      * Geel (20-50%): Matig geladen
 *      * Rood (<20%): Bijna leeg
 *    - WiFi verbindingsstatus (Verbonden/Geen)
 *
 * SCHERM 4 - BESTANDENSCHERM (ui_ScreenFiles):
 *    - Bestandsbrowser voor SD kaart
 *    - Toont bestanden met type-specifieke iconen en kleuren
 *    - Scrollbare lijst met bestanden
 *    - Tik op bestand om te openen:
 *      * Afbeeldingen (JPG/PNG/GIF/BMP): Direct weergeven
 *      * Audio (MP3/WAV/OGG/FLAC): Afspelen via ES8311 codec
 *      * Tekst (TXT/LOG/CSV): Weergeven
 *
 * ====================================================================================
 * NAVIGATIE - SCHERMEN:
 * ====================================================================================
 *
 * HORIZONTALE SWIPES (tussen schermen):
 * - SWIPE NAAR LINKS:  Ga naar het volgende scherm (Klok -> Stappen -> Instellingen -> Bestanden)
 * - SWIPE NAAR RECHTS: Ga naar het vorige scherm (Bestanden -> Instellingen -> Stappen -> Klok)
 * - Minimale swipe afstand: 50 pixels
 * - Swipe moet meer horizontaal dan verticaal zijn
 * - Animatie: 300ms slide transitie
 *
 * ====================================================================================
 * NAVIGATIE - BESTANDSBROWSER:
 * ====================================================================================
 *
 * IN HET BESTANDENSCHERM:
 * - TIK (1 seconde): Open bestand of map
 * - SCROLL: Blader door de bestandenlijst (automatisch via LVGL)
 *
 * IN DE FILE VIEWER (afbeelding/tekst/audio):
 * - SWIPE OMHOOG:    Volgende bestand van hetzelfde type
 * - SWIPE OMLAAG:    Vorige bestand van hetzelfde type
 * - SWIPE LINKS/RECHTS (alleen audio): Volume omlaag/omhoog
 * - LANG DRUKKEN (3 sec): Terug naar bestandslijst
 *
 * ONDERSTEUNDE BESTANDSTYPEN:
 * - Afbeeldingen: JPG, JPEG, PNG, GIF, BMP, PCX (groen icoon)
 * - Audio: MP3, WAV, OGG, FLAC (oranje icoon)
 * - Tekst: TXT, LOG, CSV (blauw icoon)
 * - Mappen: Tik om te openen, ".." om terug te gaan
 *
 * ====================================================================================
 * ENERGIEBESPARING:
 * ====================================================================================
 *
 * Het systeem implementeert een 3-staps power saving mechanisme:
 *
 * 1. ACTIEF (brightness = 200):
 *    - Normale werking na elke touch interactie
 *    - Alle updates actief (tijd, batterij, WiFi)
 *
 * 2. GEDIMD (brightness = 50):
 *    - Na 15 seconden zonder touch
 *    - Display nog zichtbaar maar donkerder
 *    - Alle updates blijven actief
 *
 * 3. SLAAPSTAND (brightness = 0):
 *    - Na 30 seconden zonder touch
 *    - Display volledig uit
 *    - Updates gepauzeerd om stroom te besparen
 *    - Elke touch wekt het display direct
 *
 * ====================================================================================
 * NTP TIJD SYNCHRONISATIE:
 * ====================================================================================
 *
 * - NTP Server: pool.ntp.org (configurable in wifi_config.h)
 * - Tijdzone: Europa/Brussel (CET-1CEST,M3.5.0,M10.5.0/3)
 *   * CET (Central European Time): UTC+1 (winter)
 *   * CEST (Central European Summer Time): UTC+2 (zomer)
 *   * Zomertijd start: laatste zondag van maart om 02:00
 *   * Wintertijd start: laatste zondag van oktober om 03:00
 * - Automatische retry elke 60 seconden bij mislukte sync
 * - Tijd wordt elke seconde bijgewerkt op het display
 *
 * ====================================================================================
 * MEMORY MANAGEMENT:
 * ====================================================================================
 *
 * - LVGL Draw Buffer: 1/4 van schermgrootte (102.615 bytes)
 * - Buffer wordt bij voorkeur uit PSRAM gealloceerd
 * - Fallback naar intern RAM als PSRAM niet beschikbaar is
 * - RGB565 formaat: 2 bytes per pixel
 *
 * ====================================================================================
 * LIBRARIES EN DEPENDENCIES:
 * ====================================================================================
 *
 * Vereiste libraries (installeer via Arduino Library Manager of handmatig):
 *
 * 1. LVGL (v9.3.0)                - Grafische gebruikersinterface
 * 2. Arduino_GFX_Library          - Display driver voor CO5300
 * 3. Arduino_DriveBus_Library     - I2C driver voor FT3168 touch
 * 4. XPowersLib                   - AXP2101 power management
 * 5. WiFi (ingebouwd ESP32)       - WiFi connectiviteit
 * 6. Wire (ingebouwd Arduino)     - I2C communicatie
 * 7. time (ingebouwd ESP32)       - NTP tijd synchronisatie
 *
 * ====================================================================================
 * COMPILATIE INSTELLINGEN:
 * ====================================================================================
 *
 * Board: ESP32S3 Dev Module
 * CPU Frequency: 240MHz (WiFi)
 * Flash Mode: QIO 80MHz
 * Flash Size: 16MB (128Mb)
 * Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS)
 * PSRAM: OPI PSRAM
 * Upload Speed: 921600
 * USB Mode: Hardware CDC and JTAG
 *
 * ====================================================================================
 */

// ============================================================================
// INCLUDE SECTIES - Benodigde bibliotheken
// ============================================================================
//
// Deze sectie bevat alle benodigde header files. De volgorde is belangrijk
// omdat sommige libraries afhankelijk zijn van defines in andere headers.

/*
 * Arduino.h - Arduino Core Library
 * --------------------------------
 * Bevat fundamentele Arduino functies zoals:
 * - pinMode(), digitalWrite(), digitalRead()
 * - analogRead(), analogWrite()
 * - delay(), millis(), micros()
 * - Serial communicatie
 * Deze header wordt automatisch toegevoegd in Arduino IDE maar is expliciet
 * nodig voor PlatformIO of andere development omgevingen.
 */
#include <Arduino.h>
#include <math.h>       // Voor cos() en sin() in vinyl record tekening

/*
 * lvgl.h - Light and Versatile Graphics Library (v9.3.0)
 * ------------------------------------------------------
 * Open-source embedded graphics library voor:
 * - UI widgets (labels, buttons, sliders, etc.)
 * - Animaties en transities
 * - Touch input handling
 * - Font rendering
 * - Kleurbeheer
 *
 * LVGL vereist:
 * - Een display driver (flush callback)
 * - Een input driver (touch callback)
 * - Periodieke tick updates
 * - Task handler aanroepen in loop
 *
 * Configuratie: lv_conf.h (in de LVGL library folder)
 */
#include <lvgl.h>

/*
 * Arduino_GFX_Library.h - Display Driver Library
 * ----------------------------------------------
 * Ondersteunt vele display controllers waaronder de CO5300 AMOLED.
 * Biedt een uniforme interface voor:
 * - Display initialisatie
 * - Pixel tekenen
 * - Bitmap rendering
 * - Kleurbeheer
 *
 * De library ondersteunt verschillende bus types:
 * - SPI (standaard)
 * - QSPI (Quad SPI - 4x sneller)
 * - Parallel 8/16-bit
 * - I2C (voor kleine OLED displays)
 */
#include <Arduino_GFX_Library.h>

/*
 * Wire.h - I2C Communication Library
 * ----------------------------------
 * Hardware I2C driver voor ESP32.
 * Gebruikt voor communicatie met:
 * - FT3168 Touch Controller (adres 0x38)
 * - AXP2101 Power Management IC (adres 0x34)
 *
 * I2C karakteristieken:
 * - Two-wire interface (SDA + SCL)
 * - Master-slave architectuur
 * - Meerdere slaves op dezelfde bus
 * - Clock speed: standaard 100kHz, fast mode 400kHz
 */
#include <Wire.h>

/*
 * WiFi.h - ESP32 WiFi Library
 * ---------------------------
 * Ingebouwde ESP32 WiFi stack voor:
 * - Station mode (connect to access point)
 * - Access Point mode (create own network)
 * - Scan voor beschikbare netwerken
 * - WPA/WPA2 authenticatie
 *
 * In dit project gebruikt voor:
 * - Verbinden met thuisnetwerk
 * - Internet toegang voor NTP
 */
#include <WiFi.h>

/*
 * time.h - C Standard Time Library
 * --------------------------------
 * Standaard C library voor tijd functies.
 * Op ESP32 uitgebreid met:
 * - configTzTime(): Configureer tijdzone en NTP server
 * - getLocalTime(): Haal lokale tijd op (met timezone)
 * - struct tm: Tijd structuur (uur, min, sec, dag, maand, jaar, etc.)
 *
 * NTP (Network Time Protocol):
 * - Synchroniseert klok via internet
 * - Nauwkeurigheid: typisch < 50ms
 * - Automatische zomer/wintertijd omschakeling
 */
#include <time.h>

/*
 * SPI.h - SPI Communication Library
 * ---------------------------------
 * Hardware SPI driver voor ESP32.
 * Gebruikt voor communicatie met de SD kaart.
 */
#include <SPI.h>

/*
 * SD.h - SD Card Library
 * ----------------------
 * Library voor het lezen en schrijven van SD kaarten.
 * Ondersteunt FAT16 en FAT32 bestandssystemen.
 * Gebruikt SPI interface voor communicatie.
 */
#include <SD.h>

/*
 * Audio.h - ESP32-audioI2S Library
 * --------------------------------
 * Volledige audio playback library voor ESP32.
 * Ondersteunt MP3, AAC, FLAC, WAV via I2S output.
 * Werkt met ES8311 en andere audio codecs.
 */
#include "Audio.h"

/*
 * TJpg_Decoder.h - JPEG Decoder Library
 * -------------------------------------
 * Lichtgewicht JPEG decoder voor microcontrollers.
 * Decodeert JPEG afbeeldingen block-voor-block met callbacks.
 * Efficiënt geheugengebruik door streaming decodering.
 */
#include <TJpg_Decoder.h>

/*
 * PNGdec.h - PNG Decoder Library
 * ------------------------------
 * Universele PNG decoder voor MCUs.
 * Decodeert regel-voor-regel voor minimaal geheugengebruik.
 * Ondersteunt alle PNG pixel formaten.
 */
#include <PNGdec.h>

/*
 * AnimatedGIF.h - GIF Decoder Library
 * -----------------------------------
 * Universele GIF speler voor MCUs.
 * Ondersteunt geanimeerde GIFs met meerdere frames.
 * Werkt met SD kaart, FLASH of RAM opslag.
 */
#include <AnimatedGIF.h>

/*
 * Arduino_DriveBus_Library.h - Touch Controller Driver
 * ----------------------------------------------------
 * Specifieke driver library voor de FT3168 touch controller.
 * Onderdeel van het Arduino_DriveBus framework dat drivers biedt voor:
 * - Touch controllers (FT3168, FT5x06, GT911, etc.)
 * - Sensors
 * - Andere I2C/SPI peripherals
 *
 * Features voor FT3168:
 * - Single en multi-touch detectie
 * - Interrupt-based touch events
 * - Power modes (active, monitor, hibernate)
 * - Gesture herkenning
 */
#include "Arduino_DriveBus_Library.h"

/*
 * pin_config.h - Hardware Pin Definities
 * --------------------------------------
 * Lokaal configuratiebestand met alle GPIO pin toewijzingen:
 * - Display pins (CS, SCLK, SDIO0-3, RESET)
 * - I2C pins (SDA, SCL)
 * - Touch interrupt pin
 *
 * BELANGRIJK: Dit bestand MOET voor XPowersLib.h worden geinclude
 * omdat het de XPOWERS_CHIP_AXP2101 define bevat die XPowersLib nodig heeft.
 */
#include "pin_config.h"

/*
 * wifi_config.h - WiFi en NTP Configuratie
 * ----------------------------------------
 * Lokaal configuratiebestand met:
 * - WIFI_SSID: Naam van het WiFi netwerk
 * - WIFI_PASSWORD: Wachtwoord van het WiFi netwerk
 * - NTP_SERVER: NTP server adres (bijv. "pool.ntp.org")
 * - TZ_INFO: Timezone string voor Europa/Brussel
 *
 * VEILIGHEID: Dit bestand bevat gevoelige informatie!
 * Voeg toe aan .gitignore om te voorkomen dat credentials
 * per ongeluk worden gedeeld.
 */
#include "wifi_config.h"

/*
 * XPowersLib.h - AXP2101 Power Management Library
 * -----------------------------------------------
 * Driver voor de AXP2101 Power Management Unit.
 * Functies:
 * - Batterij niveau meting (0-100%)
 * - Laadstatus detectie
 * - Voltage regulatie (DCDC, LDO outputs)
 * - Power-on/off control
 * - Interrupt handling (low battery, charging, etc.)
 *
 * De AXP2101 beheert de volledige power supply van het horloge:
 * - Batterij laden via USB
 * - 3.3V voor ESP32
 * - Display power
 * - Sensor power
 */
#include "XPowersLib.h"

/*
 * ui.h - LVGL User Interface Definities
 * -------------------------------------
 * Gegenereerd door SquareLine Studio of handmatig gemaakt.
 * Bevat:
 * - ui_init(): Initialisatie van alle schermen
 * - Screen objecten: ui_ScreenClock, ui_ScreenSteps, ui_ScreenSettings
 * - Label objecten: ui_time_label, ui_seconds_label, ui_date_label, etc.
 * - Widget objecten: sliders, buttons, etc.
 *
 * De UI is opgebouwd uit 3 schermen die via swipe navigeerbaar zijn.
 */
#include "ui.h"

// ============================================================================
// DISPLAY CONFIGURATIE CONSTANTEN
// ============================================================================
//
// Deze constanten definiëren de fysieke eigenschappen van het display.
// Ze worden gebruikt voor buffer allocatie en LVGL configuratie.

/*
 * SCREEN_WIDTH - Display breedte in pixels
 *
 * De CO5300 AMOLED heeft een resolutie van 410x502 pixels.
 * Dit is een ongebruikelijke resolutie (niet 16:9 of 4:3) omdat
 * het display ontworpen is voor smartwatch toepassingen met
 * een langwerpig formaat.
 */
#define SCREEN_WIDTH  410

/*
 * SCREEN_HEIGHT - Display hoogte in pixels
 *
 * Met 502 pixels hoogte is het display hoger dan breed,
 * ideaal voor het weergeven van tijd, notificaties en
 * verticale lijsten.
 */
#define SCREEN_HEIGHT 502

// ============================================================================
// DISPLAY HARDWARE OBJECTEN
// ============================================================================
//
// Deze sectie initialiseert de display hardware objecten.
// De objecten worden globaal aangemaakt zodat ze overal toegankelijk zijn.

/*
 * QSPI Display Bus Object
 * -----------------------
 * De CO5300 AMOLED gebruikt een QSPI (Quad SPI) interface.
 *
 * Verschil tussen SPI en QSPI:
 * - Standaard SPI: 1 data lijn (MOSI), 1 klok lijn (SCLK)
 * - QSPI: 4 data lijnen (SDIO0-3), 1 klok lijn (SCLK)
 *
 * Voordeel QSPI:
 * - 4x hogere datasnelheid bij dezelfde klokfrequentie
 * - Essentieel voor vloeiende animaties op hoge resolutie displays
 *
 * Pin toewijzing (gedefinieerd in pin_config.h):
 * - LCD_CS:    Chip Select - activeert het display
 * - LCD_SCLK:  Serial Clock - synchroniseert data
 * - LCD_SDIO0: Data lijn 0 (equivalent aan MOSI)
 * - LCD_SDIO1: Data lijn 1
 * - LCD_SDIO2: Data lijn 2
 * - LCD_SDIO3: Data lijn 3
 */
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    LCD_CS,     // Chip Select pin - LOW = display actief
    LCD_SCLK,   // Serial Clock - data wordt gesampeld op rising edge
    LCD_SDIO0,  // Data lijn 0 - LSB bij QSPI transfer
    LCD_SDIO1,  // Data lijn 1
    LCD_SDIO2,  // Data lijn 2
    LCD_SDIO3   // Data lijn 3 - MSB bij QSPI transfer
);

/*
 * CO5300 AMOLED Display Driver Object
 * -----------------------------------
 * De Arduino_CO5300 class is specifiek voor de CO5300 display controller.
 *
 * AMOLED (Active Matrix Organic LED) eigenschappen:
 * - Elke pixel is een eigen LED (geen backlight nodig)
 * - Perfect zwart (pixels zijn UIT)
 * - Hoog contrast ratio
 * - Brede kijkhoeken
 * - Lager stroomverbruik bij donkere content
 *
 * Constructor parameters:
 * - bus: Pointer naar het QSPI bus object
 * - LCD_RESET: Hardware reset pin (active LOW)
 * - rotation: Display rotatie (0, 1, 2, of 3 = 0°, 90°, 180°, 270°)
 * - width/height: Display dimensies
 * - col_offset1: Column offset - KRITIEK voor correcte pixel mapping!
 *   De CO5300 heeft een intern RAM dat groter is dan het display.
 *   De offset zorgt dat we naar de juiste geheugenlocatie schrijven.
 */
Arduino_GFX *gfx = new Arduino_CO5300(
    bus,            // QSPI data bus object voor communicatie
    LCD_RESET,      // Hardware reset pin - pulse LOW om display te resetten
    0,              // Rotatie: 0 = geen rotatie (portrait mode)
    SCREEN_WIDTH,   // Breedte: 410 pixels
    SCREEN_HEIGHT,  // Hoogte: 502 pixels
    22,             // Column offset 1 - BELANGRIJK: specifiek voor CO5300!
    0,              // Row offset 1 - geen offset nodig
    0,              // Column offset 2 - voor alternatieve rotaties
    0               // Row offset 2 - voor alternatieve rotaties
);

// ============================================================================
// TOUCH CONTROLLER CONFIGURATIE
// ============================================================================
//
// De FT3168 is een capacitieve touch controller die multi-touch ondersteunt.
// We gebruiken interrupt-driven touch detectie voor minimale latentie.

/*
 * FT3168 I2C Adres
 * ----------------
 * Het standaard I2C adres van de FT3168 is 0x38 (56 decimaal).
 * Dit adres is fabrieks-ingesteld en kan niet worden gewijzigd.
 *
 * I2C adres formaat: 7-bit adres (0x38 = 0011 1000)
 * Met R/W bit wordt dit: 0x70 (write) of 0x71 (read)
 */
#define FT3168_DEVICE_ADDRESS 0x38

// ============================================================================
// QMI8658 IMU (ACCELEROMETER + GYROSCOOP) CONFIGURATIE
// ============================================================================
//
// De QMI8658 is een 6-axis Inertial Measurement Unit (IMU) die zowel
// accelerometer als gyroscoop bevat. We gebruiken de accelerometer
// voor stappen detectie.

/*
 * QMI8658 I2C Adres en Register Definities
 * ----------------------------------------
 * De QMI8658 communiceert via I2C op adres 0x6B.
 * Registers voor configuratie en data uitlezing.
 */
#define QMI8658_ADDRESS       0x6B    // I2C adres van de QMI8658
#define QMI8658_WHO_AM_I      0x00    // Device ID register (verwacht: 0x05)
#define QMI8658_CTRL1         0x02    // Control register 1 (sensor enable)
#define QMI8658_CTRL2         0x03    // Control register 2 (accel config)
#define QMI8658_CTRL3         0x04    // Control register 3 (gyro config)
#define QMI8658_CTRL7         0x08    // Control register 7 (enable accel+gyro)
#define QMI8658_ACCEL_XOUT_L  0x35    // Accelerometer X-axis low byte

/*
 * I2C Bus Object voor Touch Controller
 * ------------------------------------
 * We gebruiken een shared_ptr voor automatisch memory management.
 * Arduino_HWIIC is een wrapper rond de hardware I2C van ESP32.
 *
 * Parameters:
 * - IIC_SDA: Data lijn (bidirectioneel)
 * - IIC_SCL: Clock lijn (master only)
 * - &Wire: Pointer naar de Wire instance (hardware I2C)
 *
 * De bus wordt gedeeld met de AXP2101 PMU (beide op dezelfde I2C bus).
 */
std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
    std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

/*
 * Forward Declaration - Touch Interrupt Handler
 * ---------------------------------------------
 * Forward declaration is nodig omdat de functie wordt doorgegeven
 * aan de FT3168 constructor voordat de functie is gedefinieerd.
 */
void Arduino_IIC_Touch_Interrupt(void);

/*
 * FileType Enum - Bestandstype classificatie
 * -----------------------------------------
 * Enum voor het classificeren van bestandstypen.
 * Moet vroeg gedefinieerd worden omdat het in functies wordt gebruikt.
 */
enum FileType { FILE_TYPE_NONE = 0, FILE_TYPE_IMAGE = 1, FILE_TYPE_TEXT = 2, FILE_TYPE_AUDIO = 3 };

/*
 * Forward Declarations - File Viewer Functions
 * -------------------------------------------
 * Nodig omdat deze functies worden aangeroepen voordat ze zijn gedefinieerd.
 */
void populateFileList();
void exitFileViewer();
void showNextFile();
void showPrevFile();
void buildFileListByType(FileType targetType);
void navigateToFolder(const char* folderPath);
void showFilesScreenError(const char* message);
FileType getFileTypeEnum(const char* filename);
bool showImageFromSD(const char *filename, bool centerX, bool centerY);
bool showGifFromSD(const char *filename, bool centerX, bool centerY);
bool showBmpFromSD(const char *filename, bool centerX, bool centerY);
bool showTextFromSD(const char *filename);
void renderTextPage();
void freeTextLines();
void fillScreenBlackPSRAM();
bool drawPngAtXY(const char* sdPath, int16_t x, int16_t y);
void showAudioPlayer(const char *filename);
void drawMusicBackground();
void drawVolumeBar(uint8_t volumePercent);
bool initAudio();
bool initES8311();
void es8311WriteReg(uint8_t reg, uint8_t val);
uint8_t es8311ReadReg(uint8_t reg);
void setAudioVolume(uint8_t volume);
void startAudioPlayback(const char* filename);
void stopAudioPlayback();
void audioLoop();
void toggleAudioPlayback();
void handleAudioTap(int16_t x, int16_t y);

/*
 * FT3168 Touch Controller Object
 * ------------------------------
 * We gebruiken een unique_ptr voor automatisch memory management.
 * De FT3168 wordt geconfigureerd met interrupt-based detection.
 *
 * Arduino_FT3x68 parameters:
 * - IIC_Bus: I2C bus object
 * - FT3168_DEVICE_ADDRESS: I2C adres (0x38)
 * - DRIVEBUS_DEFAULT_VALUE: Standaard configuratie waarden
 * - TP_INT: Touch interrupt pin - gaat LOW bij touch event
 * - Arduino_IIC_Touch_Interrupt: Callback functie voor interrupt
 *
 * Interrupt-based vs Polling:
 * - Interrupt: CPU doet andere taken, wordt gewekt bij touch
 * - Polling: CPU moet continu controleren op touch
 * - Interrupt is efficiënter voor batterij-gevoede applicaties
 */
std::unique_ptr<Arduino_IIC> FT3168(
    new Arduino_FT3x68(
        IIC_Bus,                    // Gedeelde I2C bus
        FT3168_DEVICE_ADDRESS,      // I2C adres: 0x38
        DRIVEBUS_DEFAULT_VALUE,     // Default configuratie
        TP_INT,                     // Interrupt pin (active LOW)
        Arduino_IIC_Touch_Interrupt // Interrupt callback
    )
);

/*
 * Touch Interrupt Handler (ISR - Interrupt Service Routine)
 * ---------------------------------------------------------
 * Deze functie wordt aangeroepen door de hardware wanneer de
 * TP_INT pin LOW gaat (touch event gedetecteerd).
 *
 * BELANGRIJK ISR regels:
 * 1. Houd de ISR zo kort mogelijk
 * 2. Geen blocking operaties (delay, Serial.print, etc.)
 * 3. Gebruik volatile voor gedeelde variabelen
 * 4. Geen heap allocaties (malloc, new)
 *
 * We zetten alleen een flag die in de main loop wordt verwerkt.
 * Dit patroon heet "deferred interrupt handling".
 */
void Arduino_IIC_Touch_Interrupt(void) {
    // Zet interrupt flag - wordt verwerkt in my_touchpad_read()
    FT3168->IIC_Interrupt_Flag = true;
}

// ============================================================================
// POWER MANAGEMENT
// ============================================================================
//
// De AXP2101 is een geavanceerde Power Management Unit (PMU) die
// alle stroomvoorziening van het horloge beheert.

/*
 * AXP2101 Power Management Unit Object
 * ------------------------------------
 * De XPowersPMU class biedt een abstractie laag voor de AXP2101.
 *
 * AXP2101 mogelijkheden:
 * - 4x DC-DC buck converters (hoge efficiëntie voltage regulatie)
 * - 5x LDO regulators (low-dropout voor gevoelige circuits)
 * - Li-Ion/Li-Po batterij lader
 * - Batterij niveau meting (fuel gauge)
 * - Power-on/off control via knop
 * - Temperatuur monitoring
 * - Overcurrent en overvoltage bescherming
 *
 * In dit project gebruiken we:
 * - Batterij percentage meting
 * - Batterij aanwezigheid detectie
 * - Laadstatus monitoring
 */
XPowersPMU power;

// ============================================================================
// SD KAART CONFIGURATIE
// ============================================================================
//
// MicroSD kaart ondersteuning via SPI interface.
// Pinnen zijn gedefinieerd in pin_config.h.

/*
 * SD Kaart SPI Bus
 * ----------------
 * Aparte SPI bus voor de SD kaart (HSPI).
 * Dit voorkomt conflicten met het display dat QSPI gebruikt.
 */
SPIClass sdSPI(HSPI);

/*
 * SD Kaart Status
 * ---------------
 * Houdt bij of de SD kaart succesvol is gemount.
 */
bool sdCardAvailable = false;

/*
 * SD Kaart Grootte
 * ----------------
 * Totale capaciteit van de SD kaart in bytes.
 */
uint64_t sdCardSize = 0;

// ============================================================================
// IMAGE DECODER CONFIGURATIE
// ============================================================================
//
// Objecten en variabelen voor het decoderen van afbeeldingen.

/*
 * PNG Decoder Object
 * ------------------
 * Instantie van de PNGdec library voor PNG decodering.
 */
PNG png;

/*
 * GIF Decoder Object
 * ------------------
 * Instantie van de AnimatedGIF library voor GIF decodering.
 */
AnimatedGIF gif;

/*
 * Afbeelding Positie
 * ------------------
 * X en Y offset voor het centreren van afbeeldingen op het display.
 */
int16_t imageXOffset = 0;
int16_t imageYOffset = 0;

/*
 * PNG PSRAM Buffer
 * ----------------
 * De CO5300 QSPI display ondersteunt geen draw16bitRGBBitmap met h=1
 * (enkele pixel-rijen). De QSPI writePixels() vereist grotere transfers.
 * Oplossing: buffer de hele PNG in PSRAM en teken in één keer.
 * Als pngPsramBuffer != nullptr, kopieert de callback naar de buffer.
 * Als pngPsramBuffer == nullptr, tekent de callback direct (fallback).
 */
uint16_t* pngPsramBuffer = nullptr;
int16_t pngBufferWidth = 0;
int16_t pngBufferHeight = 0;

// ============================================================================
// FILE VIEWER VARIABELEN
// ============================================================================
//
// Variabelen voor de afbeelding/tekst/audio viewer mode.

/*
 * File Viewer Mode Status
 * -----------------------
 * Houdt bij of we in viewer mode zijn (afbeelding, tekst of audio).
 * Wanneer actief worden touch events apart afgehandeld.
 */
bool fileViewerActive = false;

/*
 * Type van de viewer (voor navigatie tussen bestanden van zelfde type)
 * FileType enum is eerder gedefinieerd (zie forward declarations)
 */
FileType currentViewerType = FILE_TYPE_NONE;

/*
 * Bestandenlijst van hetzelfde type voor navigatie
 * Wordt opgebouwd bij openen van eerste bestand
 */
String fileList[64];        // Max 64 bestanden
int fileListCount = 0;      // Aantal bestanden in lijst
int currentFileIndex = 0;   // Huidige index in lijst

/*
 * Huidige directory voor folder navigatie
 * Start in root "/" en kan dieper navigeren via subfolders
 */
String currentDirectory = "/";

/*
 * Touch tracking voor viewer
 */
unsigned long viewerTouchStartTime = 0;   // Tijd van eerste touch (voor long press)
unsigned long viewerActivatedTime = 0;    // Tijd wanneer viewer werd geactiveerd
unsigned long viewerReleaseTime = 0;      // NIEUW: Tijd wanneer initiële release werd gedetecteerd
int16_t viewerTouchStartX = 0;            // Start X positie (voor links/rechts swipe)
int16_t viewerTouchStartY = 0;            // Start Y positie (voor omhoog/omlaag swipe)
bool viewerIsTouching = false;            // Touch actief in viewer
bool viewerInitialReleaseDetected = false; // Hebben we de initiële touch release gezien?
const unsigned long LONG_PRESS_TIME = 3000; // Tijd voor long press (3 seconden)
const unsigned long VIEWER_DEBOUNCE_TIME = 500; // NIEUW: Wacht 500ms na release voordat nieuwe touches geaccepteerd worden
const int16_t SWIPE_THRESHOLD = 100;       // Minimum pixels voor swipe

/*
 * Pending Viewer Open
 * -------------------
 * Uitgestelde viewer activatie - wordt verwerkt NA lv_timer_handler()
 * Dit voorkomt dat LVGL over onze gfx tekeningen rendert.
 */
bool pendingViewerOpen = false;            // Is er een uitgestelde viewer open?
FileType pendingViewerType = FILE_TYPE_NONE;  // Type van pending viewer
String pendingViewerFile = "";             // Bestandspad van pending viewer

// ============================================================================
// TEKST VIEWER SCROLL VARIABELEN
// ============================================================================
//
// Voor scrollen binnen een tekstbestand. Alle regels worden bij openen
// in PSRAM opgeslagen zodat we snel kunnen hertekenen bij scrollen.

#define TEXT_FONT_SIZE       2     // Lettergrootte (2x = 10x14 pixels per karakter)
#define TEXT_LINE_HEIGHT     18    // Pixels per regel (16px karakter + 2px gap)
#define TEXT_HEADER_Y        40    // Y start positie van tekstinhoud (na header)
#define TEXT_FOOTER_HEIGHT   28    // Ruimte onderaan voor pagina-info
#define TEXT_CHARS_PER_LINE  32    // Zichtbare karakters per regel bij size 2
#define TEXT_CHAR_WIDTH      12    // Pixels per karakter bij size 2 (6px * 2)
#define TEXT_H_SCROLL_STEP   10    // Karakters per horizontale scroll stap
#define TEXT_MAX_LINES       500   // Max aantal regels in geheugen
#define TEXT_MAX_LINE_LEN    200   // Max opgeslagen karakters per regel

int textScrollLine = 0;                    // Eerste zichtbare regel (verticale scroll)
int textScrollCol = 0;                     // Eerste zichtbare kolom (horizontale scroll)
int textMaxCol = 0;                        // Langste regel in het bestand (in karakters)
int textTotalLines = 0;                    // Totaal aantal regels in het bestand
int textLinesPerPage = 0;                  // Regels per pagina (berekend)
String *textFileLines = nullptr;           // Dynamisch array met alle regels (PSRAM)
String textCurrentFile = "";               // Huidig geopend tekstbestand

// ============================================================================
// AUDIO VARIABELEN
// ============================================================================
//
// Variabelen voor audio playback met ES8311 codec en ESP32-audioI2S library.

/*
 * Audio Object
 * ------------
 * Hoofd audio object van de ESP32-audioI2S library.
 * Ondersteunt MP3, AAC, FLAC, WAV, OGG playback.
 */
Audio audio;

/*
 * Audio Status
 * ------------
 * Houdt bij of audio speelt en welk bestand.
 */
bool audioAvailable = false;
bool audioPlaying = false;
bool audioPaused = false;            // true = audio is gepauzeerd
String currentAudioFile = "";

/*
 * Audio Volume
 * ------------
 * Volume level van 0-21 (library max)
 */
uint8_t audioVolume = 15;  // Default volume (0-21)

// ============================================================================
// QMI8658 IMU VARIABELEN
// ============================================================================
//
// Variabelen voor de accelerometer en stappen detectie.

/*
 * IMU Status
 * ----------
 * Houdt bij of de QMI8658 succesvol is geïnitialiseerd.
 */
bool imuAvailable = false;

/*
 * Accelerometer Data
 * ------------------
 * Huidige acceleratie waarden in g (gravity units).
 * Bereik: -4g tot +4g (bij huidige configuratie)
 */
float accelX = 0, accelY = 0, accelZ = 0;

/*
 * Stappen Detectie Algoritme Variabelen
 * -------------------------------------
 * Low-pass filter en piek-dal detectie voor nauwkeurige stappen telling.
 */
// Low-pass filter voor ruisonderdrukking
float filteredMagnitude = 1.0;           // Gefilterde magnitude (start bij 1g = rust)
const float filterAlpha = 0.1;           // Filter coefficient (lager = meer smoothing)

// Piek-dal detectie state machine
float peakValue = 0;                     // Huidige piek waarde
float valleyValue = 10;                  // Huidige dal waarde
bool lookingForPeak = true;              // State: zoeken naar piek of dal

// Stap detectie thresholds (VERDER VERHOOGD voor nog nauwkeurigere telling)
// 20 stappen gaf 27 (1.35x), dus thresholds met 40% verhoogd
const float stepThresholdHigh = 1.00;    // Minimum piek hoogte voor geldige stap (was 0.80)
const float stepThresholdLow = 0.50;     // Minimum verschil piek-dal (was 0.40)
const float noiseFloor = 0.20;           // Onder deze waarde = ruis, negeren (was 0.15)
const int minStepInterval = 420;         // Minimum tijd tussen stappen (ms) - max 142 stappen/min
const int maxStepInterval = 800;         // Maximum tijd tussen stappen (ms) - sneller stoppen bij stilstaan

// Timing variabelen
unsigned long lastStepTime = 0;          // Tijdstip van laatste gedetecteerde stap
unsigned long lastPeakTime = 0;          // Tijdstip van laatste piek
unsigned long lastAccelRead = 0;         // Tijdstip van laatste accelerometer uitlezing

// Stap validatie (voorkomt valse positieven)
int pendingSteps = 0;                    // Stappen wachtend op validatie
int consecutiveValidSteps = 0;           // Aantal opeenvolgende geldige stappen
const int stepsToValidate = 5;           // Vereiste stappen om walking mode te starten (verhoogd)
unsigned long lastPendingStepTime = 0;   // Tijdstip laatste pending stap
bool isWalking = false;                  // Walking mode actief

// ============================================================================
// LVGL DISPLAY BUFFER CONFIGURATIE
// ============================================================================
//
// LVGL vereist een buffer voor het renderen van pixels voordat
// ze naar het fysieke display worden gestuurd.

/*
 * Display Driver Pointer
 * ----------------------
 * Pointer naar het LVGL display driver object.
 * Dit object koppelt LVGL aan ons fysieke display.
 */
static lv_display_t *lvgl_disp;

/*
 * Input Device Driver Pointer
 * ---------------------------
 * Pointer naar het LVGL input device (touchpad) driver object.
 * Dit object koppelt LVGL aan onze touch controller.
 */
static lv_indev_t *lvgl_indev;

/*
 * Bytes Per Pixel
 * ---------------
 * RGB565 kleurformaat gebruikt 16 bits (2 bytes) per pixel.
 *
 * RGB565 bit layout:
 * RRRR RGGG GGGB BBBB
 * - R: 5 bits rood (0-31)
 * - G: 6 bits groen (0-63) - extra bit omdat oog gevoeliger is voor groen
 * - B: 5 bits blauw (0-31)
 *
 * Totaal: 65.536 verschillende kleuren
 */
#define BYTES_PER_PIXEL 2

/*
 * Draw Buffer Size
 * ----------------
 * FULL SCREEN BUFFER: We gebruiken een buffer groot genoeg voor het hele scherm.
 * Dit is nodig voor FULL render mode die partial update problemen voorkomt.
 *
 * Berekening voor volledig scherm:
 * 410 * 502 * 2 = 411.640 bytes per buffer
 *
 * We gebruiken SINGLE buffering om geheugen te besparen:
 * Totaal: 1 x 411.640 bytes = ~402KB (past in PSRAM)
 *
 * FULL render mode voorkomt de "italic" tekst vervormingen die optreden
 * bij partial updates door altijd het hele scherm te hertekenen.
 */
#define DRAW_BUF_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT * BYTES_PER_PIXEL)

/*
 * Draw Buffer Pointers
 * --------------------
 * Twee buffers voor double buffering.
 * Beide buffers worden uit PSRAM gealloceerd.
 */
static uint8_t *draw_buf1;
static uint8_t *draw_buf2;

// ============================================================================
// GLOBALE VARIABELEN - APPLICATIE STATUS
// ============================================================================
//
// Deze variabelen houden de huidige staat van de applicatie bij.
// Ze zijn globaal zodat ze vanuit alle functies toegankelijk zijn.

/*
 * Stappenteller Status
 * --------------------
 * De stappenteller is een eenvoudige demonstratie feature.
 * In een echte implementatie zou deze gekoppeld zijn aan een
 * accelerometer (bijv. LSM6DS3) voor echte stapdetectie.
 */
bool stepCountingEnabled = true;   // Is het tellen actief? true = ja
uint32_t stepCount = 0;            // Huidige telling (32-bit voor grote aantallen)

/*
 * Display Status
 * --------------
 * Houdt de huidige display instellingen bij.
 */
uint8_t currentBrightness = 200;   // Helderheid: 0 (uit) tot 255 (max)
int currentScreen = 0;             // Actief scherm: 0=klok, 1=stappen, 2=instellingen

// ============================================================================
// POWER SAVING CONFIGURATIE
// ============================================================================
//
// Power saving is essentieel voor een smartwatch met beperkte batterij.
// Het systeem volgt een 3-fase model: Actief -> Gedimd -> Slaapstand

/*
 * Laatste Activiteit Tijdstip
 * ---------------------------
 * Slaat op wanneer de gebruiker voor het laatst het scherm aanraakte.
 * Gebruikt millis() waarde (milliseconden sinds opstart).
 */
unsigned long lastActivityTime = 0;

/*
 * Display Status Flags
 * --------------------
 * Houden bij in welke power saving state het display zich bevindt.
 * Alleen één kan true zijn tegelijk (of beide false = actief).
 */
bool displayDimmed = false;   // true = display is gedimd
bool displayAsleep = false;   // true = display is uit

/*
 * Timeout Waarden (in milliseconden)
 * ----------------------------------
 * Configureerbare timeouts voor power saving transitions.
 *
 * DIM_TIMEOUT: Tijd tot dimmen
 * - 15 seconden is een goede balans tussen gebruiksgemak en batterijbesparing
 * - Gebruiker kan nog steeds zien dat het horloge aan is
 *
 * SLEEP_TIMEOUT: Tijd tot slaapstand
 * - 30 seconden geeft gebruiker tijd om naar gedimd scherm te kijken
 * - Display gaat volledig uit voor maximale besparing
 */
const unsigned long DIM_TIMEOUT = 15000;    // 15.000 ms = 15 seconden
const unsigned long SLEEP_TIMEOUT = 30000;  // 30.000 ms = 30 seconden

/*
 * Helderheid Niveaus
 * ------------------
 * Gedefinieerde helderheid levels voor verschillende states.
 *
 * DIM_BRIGHTNESS: 50 van 255 = ~20% helderheid
 * - Nog steeds leesbaar in normale verlichting
 * - Significant lager stroomverbruik
 *
 * NORMAL_BRIGHTNESS: 200 van 255 = ~78% helderheid
 * - Comfortabel leesbaar in de meeste situaties
 * - Niet maximaal om batterij te sparen en oogcomfort
 */
const uint8_t DIM_BRIGHTNESS = 50;
const uint8_t NORMAL_BRIGHTNESS = 200;

// ============================================================================
// NTP TIJD SYNCHRONISATIE VARIABELEN
// ============================================================================
//
// NTP (Network Time Protocol) synchroniseert de klok met internet tijdservers.

/*
 * NTP Sync Status Flag
 * --------------------
 * Houdt bij of de tijd succesvol is gesynchroniseerd.
 * Bij false zal het systeem periodiek opnieuw proberen.
 */
bool ntpSynced = false;

/*
 * Laatste NTP Poging Tijdstip
 * ---------------------------
 * Voorkomt te frequente NTP requests.
 * We wachten minimaal 60 seconden tussen pogingen.
 */
unsigned long lastNtpAttempt = 0;

// ============================================================================
// TOUCH TRACKING VARIABELEN
// ============================================================================
//
// Deze variabelen worden gebruikt voor touch positie tracking en gesture detectie.

/*
 * Huidige Touch Positie
 * ---------------------
 * De laatste bekende X,Y coordinaten van de touch.
 * Wordt bijgewerkt bij elke touch interrupt.
 *
 * Coordinaten systeem:
 * - (0,0) = linksboven
 * - X: 0 tot SCREEN_WIDTH-1 (0-409)
 * - Y: 0 tot SCREEN_HEIGHT-1 (0-501)
 */
int32_t touchX = 0, touchY = 0;

/*
 * Touch Start Positie
 * -------------------
 * De positie waar de touch begon (finger down).
 * Gebruikt om swipe richting en afstand te berekenen.
 */
int32_t touchStartX = 0, touchStartY = 0;

/*
 * Touch State Flags
 * -----------------
 * isTouching: Is er momenteel een vinger op het scherm?
 *   - true van finger-down tot finger-up
 *
 * touchHandled: Is de huidige gesture al verwerkt?
 *   - Voorkomt dat een swipe meerdere keren wordt getriggerd
 *   - Reset bij nieuwe touch
 */
bool isTouching = false;
bool touchHandled = false;

/*
 * Dubbel-Tik Detectie (voor scrollen in bestanden scherm)
 * -------------------------------------------------------
 * Gebruikt om dubbel-tik te detecteren voor het scrollen van de bestandenlijst.
 * - Tik bovenaan scherm = scroll omhoog
 * - Tik onderaan scherm = scroll omlaag
 */
unsigned long lastTapTime = 0;           // Tijdstip van laatste tik
int16_t lastTapY = 0;                    // Y positie van laatste tik
const unsigned long DOUBLE_TAP_TIME = 400;  // Max tijd tussen tikken (ms)
const int16_t TAP_MOVE_THRESHOLD = 30;   // Max beweging voor een tik (pixels)

// ============================================================================
// NEDERLANDSE LOKALISATIE - DAG- EN MAANDNAMEN
// ============================================================================
//
// Arrays met Nederlandse namen voor weergave op het klokscherm.
// Dit maakt de interface gebruiksvriendelijker voor Nederlandse gebruikers.

/*
 * Dag Namen Array
 * ---------------
 * Index volgens C standaard tm_wday:
 * 0 = Zondag (Sunday)
 * 1 = Maandag (Monday)
 * 2 = Dinsdag (Tuesday)
 * 3 = Woensdag (Wednesday)
 * 4 = Donderdag (Thursday)
 * 5 = Vrijdag (Friday)
 * 6 = Zaterdag (Saturday)
 *
 * LET OP: De week begint met zondag in C, niet maandag!
 */
const char* dagNamen[] = {
    "Zondag",     // tm_wday = 0
    "Maandag",    // tm_wday = 1
    "Dinsdag",    // tm_wday = 2
    "Woensdag",   // tm_wday = 3
    "Donderdag",  // tm_wday = 4
    "Vrijdag",    // tm_wday = 5
    "Zaterdag"    // tm_wday = 6
};

/*
 * Maand Namen Array
 * -----------------
 * Index volgens C standaard tm_mon:
 * 0 = Januari (January)
 * 1 = Februari (February)
 * ... etc ...
 * 11 = December
 *
 * LET OP: tm_mon is 0-based (januari = 0), niet 1-based!
 */
const char* maandNamen[] = {
    "Januari",    // tm_mon = 0
    "Februari",   // tm_mon = 1
    "Maart",      // tm_mon = 2
    "April",      // tm_mon = 3
    "Mei",        // tm_mon = 4
    "Juni",       // tm_mon = 5
    "Juli",       // tm_mon = 6
    "Augustus",   // tm_mon = 7
    "September",  // tm_mon = 8
    "Oktober",    // tm_mon = 9
    "November",   // tm_mon = 10
    "December"    // tm_mon = 11
};

// ============================================================================
// I2C HELPER FUNCTIES
// ============================================================================
//
// Low-level I2C communicatie functies voor de QMI8658 accelerometer.

/*
 * Leest een enkel register van een I2C device.
 */
uint8_t readRegister(uint8_t deviceAddress, uint8_t reg) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(reg);
    Wire.endTransmission(false);  // Repeated start
    Wire.requestFrom(deviceAddress, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

/*
 * Schrijft een waarde naar een register van een I2C device.
 */
void writeRegister(uint8_t deviceAddress, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

/*
 * Leest meerdere opeenvolgende registers van een I2C device.
 */
void readRegisters(uint8_t deviceAddress, uint8_t reg, uint8_t *buffer, uint8_t len) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(reg);
    Wire.endTransmission(false);  // Repeated start
    Wire.requestFrom(deviceAddress, len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) {
        buffer[i] = Wire.read();
    }
}

// ============================================================================
// QMI8658 IMU FUNCTIES
// ============================================================================

/*
 * Initialiseert de QMI8658 accelerometer.
 * Configureert voor ±4g bereik en 250Hz sample rate.
 *
 * Returns: true als initialisatie succesvol, false bij fout.
 */
bool initIMU() {
    Serial.println("[IMU] QMI8658 initialiseren...");

    // Lees WHO_AM_I register om device te verifiëren
    uint8_t whoAmI = readRegister(QMI8658_ADDRESS, QMI8658_WHO_AM_I);
    Serial.print("[IMU] WHO_AM_I: 0x");
    Serial.println(whoAmI, HEX);

    // QMI8658 moet 0x05 returnen
    if (whoAmI != 0x05) {
        Serial.println("[IMU] ERROR: QMI8658 niet gevonden!");
        return false;
    }

    // Software reset
    writeRegister(QMI8658_ADDRESS, QMI8658_CTRL1, 0x80);
    delay(50);

    // Configureer accelerometer: ±4g, 250Hz ODR
    // CTRL2: [7:4]=ODR (0101=250Hz), [3:1]=FS (010=±4g), [0]=self-test
    writeRegister(QMI8658_ADDRESS, QMI8658_CTRL2, 0x54);
    delay(10);

    // Enable accelerometer en gyroscoop
    // CTRL7: [1]=aEN (accel enable), [0]=gEN (gyro enable)
    writeRegister(QMI8658_ADDRESS, QMI8658_CTRL7, 0x03);
    delay(10);

    Serial.println("[IMU] QMI8658 succesvol geïnitialiseerd");
    return true;
}

/*
 * Leest de huidige acceleratie waarden van de QMI8658.
 * Waarden worden opgeslagen in globale variabelen accelX, accelY, accelZ.
 */
void readAccelerometer() {
    uint8_t data[6];
    readRegisters(QMI8658_ADDRESS, QMI8658_ACCEL_XOUT_L, data, 6);

    // Combineer low en high bytes tot 16-bit signed integers
    int16_t rawX = (int16_t)((data[1] << 8) | data[0]);
    int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
    int16_t rawZ = (int16_t)((data[5] << 8) | data[4]);

    // Converteer naar g (bij ±4g: 8192 LSB/g)
    accelX = rawX / 8192.0;
    accelY = rawY / 8192.0;
    accelZ = rawZ / 8192.0;
}

/*
 * Detecteert stappen gebaseerd op accelerometer data.
 * Gebruikt een piek-dal detectie algoritme met validatie.
 */
void detectStep() {
    // Skip als tellen uitgeschakeld is
    if (!stepCountingEnabled) return;

    // Bereken magnitude van acceleratie vector
    float magnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

    // Low-pass filter toepassen voor ruisonderdrukking
    // Verlaagd filterAlpha voor meer smoothing (minder gevoelig voor ruis)
    const float smoothingAlpha = 0.08;  // Was 0.1, nu meer smoothing
    filteredMagnitude = smoothingAlpha * magnitude + (1.0 - smoothingAlpha) * filteredMagnitude;

    // Dynamische acceleratie (afwijking van rust = 1g)
    float dynamicAccel = abs(filteredMagnitude - 1.0);

    // NOISE GATE: Negeer zeer kleine bewegingen (stilstaan)
    // Dit voorkomt valse positieven bij trillingen of kleine bewegingen
    if (dynamicAccel < noiseFloor) {
        // Reset peak tracking als we in de noise zone zijn
        if (peakValue > 0 && peakValue < stepThresholdHigh) {
            peakValue = 0;  // Geen geldige peak, reset
        }
        return;  // Negeer deze sample
    }

    unsigned long currentTime = millis();
    unsigned long timeSinceLastPeak = currentTime - lastPeakTime;

    // State machine voor piek-dal detectie
    if (lookingForPeak) {
        // Update piek waarde als we hoger gaan
        if (dynamicAccel > peakValue) {
            peakValue = dynamicAccel;
        }
        // Piek gevonden als we significant dalen EN genoeg tijd sinds laatste piek
        else if (dynamicAccel < (peakValue - stepThresholdLow) &&
                 peakValue > stepThresholdHigh &&
                 timeSinceLastPeak >= minStepInterval) {

            lookingForPeak = false;
            valleyValue = dynamicAccel;

            unsigned long timeSinceLastPending = currentTime - lastPendingStepTime;

            // Check of interval binnen loopbereik valt
            bool intervalOK = (pendingSteps == 0) ||
                              (timeSinceLastPending >= minStepInterval &&
                               timeSinceLastPending <= maxStepInterval);

            if (intervalOK) {
                pendingSteps++;
                consecutiveValidSteps++;
                lastPendingStepTime = currentTime;
                lastPeakTime = currentTime;

                // Na validatie: walking mode aan
                if (!isWalking && consecutiveValidSteps >= stepsToValidate) {
                    isWalking = true;
                    stepCount += pendingSteps;
                    Serial.print("[Stappen] Walking gestart! +");
                    Serial.print(pendingSteps);
                    Serial.print(" Totaal: ");
                    Serial.println(stepCount);
                    pendingSteps = 0;
                }
                // In walking mode: tel elke geldige stap
                else if (isWalking) {
                    stepCount++;
                    pendingSteps = 0;
                }
            } else {
                // Interval te lang of te kort - reset
                pendingSteps = 1;
                consecutiveValidSteps = 1;
                lastPendingStepTime = currentTime;
                lastPeakTime = currentTime;
                isWalking = false;
            }

            peakValue = 0;
        }
    }
    else {
        // Zoeken naar dal
        if (dynamicAccel < valleyValue) {
            valleyValue = dynamicAccel;
        }
        else if (dynamicAccel > (valleyValue + stepThresholdLow)) {
            lookingForPeak = true;
            peakValue = dynamicAccel;
            valleyValue = 10;
        }
    }

    // Stop walking mode als te lang geen stap
    if (isWalking && (currentTime - lastPeakTime) > maxStepInterval) {
        isWalking = false;
        pendingSteps = 0;
        consecutiveValidSteps = 0;
    }

    // Reset pending als niet walking en timeout
    if (!isWalking && pendingSteps > 0 && (currentTime - lastPendingStepTime) > maxStepInterval) {
        pendingSteps = 0;
        consecutiveValidSteps = 0;
    }
}

/*
 * Update het stappen display met de huidige telling.
 */
void updateStepDisplay() {
    static uint32_t lastDisplayedSteps = 0xFFFFFFFF;  // Force eerste update

    if (stepCount != lastDisplayedSteps) {
        lastDisplayedSteps = stepCount;

        static char stepBuf[12];
        snprintf(stepBuf, sizeof(stepBuf), "%lu", stepCount);
        lv_label_set_text(ui_steps_count, stepBuf);

        // Forceer hertekenen van het label
        lv_obj_invalidate(ui_steps_count);
    }
}

// ============================================================================
// FUNCTIE: setDisplayBrightness
// ============================================================================
/*
 * Stelt de helderheid van het AMOLED display in.
 *
 * Parameters:
 *   brightness - Helderheidswaarde van 0 (uit) tot 255 (maximaal)
 *
 * Technische details:
 * ------------------
 * Het CO5300 AMOLED display ondersteunt het MIPI DCS (Display Command Set)
 * standaard commando 0x51 voor helderheidsregeling.
 *
 * MIPI DCS commando 0x51: Write Display Brightness
 * - Parameter: 1 byte (0x00-0xFF)
 * - 0x00 = minimale helderheid (maar niet helemaal uit)
 * - 0xFF = maximale helderheid
 *
 * De functie gebruikt de SPI bus om het commando te versturen:
 * 1. beginWrite() - Start SPI transactie, zet CS LOW
 * 2. writeC8D8() - Stuur command byte (0x51) gevolgd door data byte
 * 3. endWrite() - Beëindig transactie, zet CS HIGH
 *
 * Stroomverbruik:
 * - AMOLED stroomverbruik schaalt lineair met helderheid
 * - Bij 50% helderheid: ~50% van max stroomverbruik voor display
 * - Bij 0%: pixels zijn uit, minimaal verbruik
 */
void setDisplayBrightness(uint8_t brightness) {
    // Start SPI transactie - CS pin gaat LOW
    bus->beginWrite();

    // Stuur MIPI DCS commando 0x51 met helderheidswaarde
    // writeC8D8 = write Command (8-bit) + Data (8-bit)
    bus->writeC8D8(0x51, brightness);

    // Beëindig SPI transactie - CS pin gaat HIGH
    bus->endWrite();

    // Sla huidige helderheid op voor latere referentie
    currentBrightness = brightness;
}

// ============================================================================
// FUNCTIE: registerActivity
// ============================================================================
/*
 * Registreert gebruikersactiviteit voor het power saving systeem.
 * Wordt aangeroepen bij elke touch interactie.
 *
 * Werking:
 * --------
 * 1. Update de lastActivityTime naar huidige tijd
 * 2. Als display in slaapstand is -> wake up naar normale helderheid
 * 3. Als display gedimd is -> herstel naar normale helderheid
 *
 * Deze functie is de "wake-up" trigger voor het power saving systeem.
 * Elke aanraking van het scherm roept deze functie aan, waardoor
 * de timeouts worden gereset en het display indien nodig wordt gewekt.
 *
 * Flow diagram:
 *
 *   Touch Event
 *        |
 *        v
 *   registerActivity()
 *        |
 *        +--> Update lastActivityTime
 *        |
 *        +--> displayAsleep?
 *        |         |
 *        |    Yes  v
 *        |    Wake up: brightness = 200
 *        |    Reset beide flags
 *        |         |
 *        |    No   v
 *        +--> displayDimmed?
 *                  |
 *             Yes  v
 *             Un-dim: brightness = 200
 *             Reset dimmed flag
 */
void registerActivity() {
    // Sla huidige tijd op als laatste activiteit
    // millis() geeft milliseconden sinds boot
    lastActivityTime = millis();

    // Check of display in slaapstand is (brightness = 0)
    if (displayAsleep) {
        // Reset beide power saving flags
        displayAsleep = false;
        displayDimmed = false;

        // Herstel normale helderheid
        setDisplayBrightness(NORMAL_BRIGHTNESS);

        // Debug output naar Serial monitor
        Serial.println("[Power] Display wakker gemaakt uit slaapstand");
    }
    // Check of display gedimd is (brightness = 50)
    else if (displayDimmed) {
        // Reset dim flag
        displayDimmed = false;

        // Herstel normale helderheid
        setDisplayBrightness(NORMAL_BRIGHTNESS);

        // Debug output
        Serial.println("[Power] Display uit dim-modus gehaald");
    }
    // Anders: display was al actief, geen actie nodig
}

// ============================================================================
// FUNCTIE: updatePowerSaving
// ============================================================================
/*
 * Beheert de power saving state machine gebaseerd op inactiviteit.
 * Wordt elke loop() iteratie aangeroepen (elke ~5ms).
 *
 * State Machine:
 * -------------
 *
 *   +---------+    15s timeout    +---------+    30s timeout    +------------+
 *   | ACTIEF  | ----------------> | GEDIMD  | ----------------> | SLAAPSTAND |
 *   | (200)   |                   | (50)    |                   | (0)        |
 *   +---------+                   +---------+                   +------------+
 *        ^                              ^                              |
 *        |          touch event         |         touch event          |
 *        +------------------------------+------------------------------+
 *
 * States:
 * - ACTIEF: displayDimmed=false, displayAsleep=false, brightness=200
 * - GEDIMD: displayDimmed=true, displayAsleep=false, brightness=50
 * - SLAAPSTAND: displayDimmed=true, displayAsleep=true, brightness=0
 *
 * Timing:
 * - De functie berekent hoelang er geen activiteit is geweest
 * - Bij 15+ seconden: transitie naar GEDIMD
 * - Bij 30+ seconden: transitie naar SLAAPSTAND
 *
 * Energie besparing:
 * - AMOLED verbruikt stroom proportioneel aan helderheid
 * - GEDIMD bespaart ~75% display energie
 * - SLAAPSTAND bespaart ~100% display energie
 */
void updatePowerSaving() {
    // Haal huidige tijd op
    unsigned long now = millis();

    // Bereken tijd sinds laatste activiteit
    unsigned long inactiveTime = now - lastActivityTime;

    // Check voor dim timeout (15 seconden)
    // Alleen als nog niet gedimd EN nog niet in slaapstand
    if (!displayAsleep && !displayDimmed && inactiveTime >= DIM_TIMEOUT) {
        // Transitie naar GEDIMD state
        displayDimmed = true;

        // Verlaag helderheid naar dim niveau
        setDisplayBrightness(DIM_BRIGHTNESS);

        // Debug output
        Serial.println("[Power] Display gedimd wegens inactiviteit");
    }

    // Check voor sleep timeout (30 seconden)
    // Alleen als nog niet in slaapstand (wel of niet gedimd)
    if (!displayAsleep && inactiveTime >= SLEEP_TIMEOUT) {
        // Transitie naar SLAAPSTAND state
        displayAsleep = true;
        // displayDimmed blijft true (impliciete state)

        // Zet display uit (brightness 0)
        setDisplayBrightness(0);

        // Debug output
        Serial.println("[Power] Display in slaapstand wegens inactiviteit");
    }
}

// ============================================================================
// FUNCTIE: my_disp_flush (LVGL Display Callback)
// ============================================================================
/*
 * LVGL display flush callback functie.
 * Dit is de brug tussen LVGL's rendering engine en ons fysieke display.
 *
 * Parameters:
 *   disp   - Pointer naar het LVGL display object
 *   area   - Struct met het rechthoekige gebied dat bijgewerkt moet worden:
 *            - area->x1, area->y1: linker-boven hoek
 *            - area->x2, area->y2: rechter-onder hoek
 *   px_map - Pointer naar de pixel data buffer in RGB565 formaat
 *
 * Werking:
 * --------
 * LVGL tekent UI elementen in zijn interne buffer. Wanneer een deel
 * van het scherm moet worden bijgewerkt, roept LVGL deze callback aan
 * met de pixel data die naar het display gestuurd moet worden.
 *
 * De functie:
 * 1. Berekent de breedte en hoogte van het update gebied
 * 2. Stuurt de pixels naar het display via draw16bitRGBBitmap()
 * 3. Informeert LVGL dat de flush compleet is
 *
 * Stap 3 is KRITIEK! Zonder lv_display_flush_ready() zal LVGL
 * wachten op de flush en de UI zal bevriezen.
 *
 * Performance:
 * - Deze functie wordt vaak aangeroepen (elke lv_timer_handler())
 * - De QSPI interface zorgt voor snelle data overdracht
 * - Partial updates (alleen gewijzigde gebieden) minimaliseren werk
 */
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    // Bereken dimensies van het update gebied
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    // Simpele bitmap draw - terug naar werkende basis
    // draw16bitRGBBitmap tekent de volledige bitmap in één keer
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);

    // Informeer LVGL dat flush compleet is
    lv_display_flush_ready(disp);
}

// ============================================================================
// FUNCTIE: my_touchpad_read (LVGL Touch Callback)
// ============================================================================
/*
 * LVGL touchpad read callback functie.
 * Wordt periodiek door LVGL aangeroepen om touch status te lezen.
 *
 * Parameters:
 *   indev - Pointer naar het LVGL input device object (ongebruikt)
 *   data  - Struct waarin we de touch data moeten invullen:
 *           - data->point.x: X coordinaat
 *           - data->point.y: Y coordinaat
 *           - data->state: LV_INDEV_STATE_PRESSED of LV_INDEV_STATE_RELEASED
 *
 * Werking:
 * --------
 * 1. Check of er een touch interrupt was (FT3168 heeft touch gedetecteerd)
 * 2. Zo ja: lees touch data via I2C
 *    - Aantal touch punten
 *    - X,Y coordinaten
 * 3. Bij touch press: sla startpositie op voor swipe detectie
 * 4. Bij touch release: check voor swipe gesture
 *
 * Swipe Detectie:
 * - Minimum swipe afstand: 50 pixels
 * - Swipe moet meer horizontaal dan verticaal zijn
 * - Swipe links (deltaX < 0): volgende scherm
 * - Swipe rechts (deltaX > 0): vorige scherm
 *
 * Touch Coordinaten:
 * - FT3168 rapporteert in display coordinaten (0-409, 0-501)
 * - Geen conversie nodig omdat display niet geroteerd is
 */
void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
    // OPMERKING: Deze functie wordt ALLEEN aangeroepen wanneer fileViewerActive = false
    // In viewer mode wordt handleViewerTouch() gebruikt in plaats van deze LVGL callback

    // Check of er een touch interrupt was
    // De interrupt handler zet deze flag wanneer TP_INT LOW gaat
    if (FT3168->IIC_Interrupt_Flag) {
        // Reset de interrupt flag voor volgende detectie
        FT3168->IIC_Interrupt_Flag = false;

        // Lees aantal actieve touch punten (0 = geen touch, 1+ = touch)
        uint8_t touchPoints = FT3168->IIC_Read_Device_Value(
            FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

        // Als er touch punten zijn (vinger op scherm)
        if (touchPoints > 0) {
            // Lees X coordinaat van eerste touch punt
            touchX = FT3168->IIC_Read_Device_Value(
                FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);

            // Lees Y coordinaat van eerste touch punt
            touchY = FT3168->IIC_Read_Device_Value(
                FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

            // Registreer activiteit voor power saving (wake up indien nodig)
            registerActivity();

            // Vul LVGL data struct met touch informatie
            data->point.x = touchX;
            data->point.y = touchY;
            data->state = LV_INDEV_STATE_PRESSED;  // Vinger is op scherm

            // Bij NIEUWE touch (was niet touching), sla startpositie op
            if (!isTouching) {
                isTouching = true;           // Markeer dat we nu touchen
                touchStartX = touchX;        // Sla start X op
                touchStartY = touchY;        // Sla start Y op
                touchHandled = false;        // Reset gesture verwerking flag
            }
        } else {
            // Geen touch punten = vinger is losgelaten
            data->state = LV_INDEV_STATE_RELEASED;

            // Bij RELEASE, check voor gestures
            if (isTouching && !touchHandled) {
                // Bereken verplaatsing van start naar eind positie
                int32_t deltaX = touchX - touchStartX;  // Positief = naar rechts
                int32_t deltaY = touchY - touchStartY;  // Positief = naar beneden
                unsigned long now = millis();

                // Check voor horizontale swipe:
                // - Minimaal 50 pixels verplaatsing
                // - Meer horizontaal dan verticaal (abs(deltaX) > abs(deltaY))
                if (abs(deltaX) > 50 && abs(deltaX) > abs(deltaY)) {
                    if (deltaX < 0) {
                        // Swipe naar LINKS (negatieve X) = volgend scherm
                        nextScreen();
                        Serial.println("[Touch] Swipe links -> volgend scherm");
                    } else {
                        // Swipe naar RECHTS (positieve X) = vorig scherm
                        prevScreen();
                        Serial.println("[Touch] Swipe rechts -> vorig scherm");
                    }
                    touchHandled = true;
                }
                // Check voor TAP (kleine beweging) - voor dubbel-tik detectie
                else if (abs(deltaX) < TAP_MOVE_THRESHOLD && abs(deltaY) < TAP_MOVE_THRESHOLD) {
                    // Dit is een tik (geen swipe)
                    // Check of we op het bestanden scherm zijn
                    if (lv_scr_act() == ui_ScreenFiles) {
                        // Check voor dubbel-tik
                        if (now - lastTapTime < DOUBLE_TAP_TIME) {
                            // DUBBEL-TIK gedetecteerd!
                            // Scroll richting hangt af van Y positie
                            int screenHeight = 502;  // Display hoogte
                            int scrollAmount = 100;  // Pixels om te scrollen

                            if (touchY < screenHeight / 3) {
                                // Tik in bovenste 1/3 = scroll omhoog (toon eerdere bestanden)
                                lv_obj_scroll_by(ui_files_list, 0, scrollAmount, LV_ANIM_ON);
                                Serial.println("[Touch] Dubbel-tik boven - scroll omhoog");
                            } else if (touchY > (screenHeight * 2) / 3) {
                                // Tik in onderste 1/3 = scroll omlaag (toon latere bestanden)
                                lv_obj_scroll_by(ui_files_list, 0, -scrollAmount, LV_ANIM_ON);
                                Serial.println("[Touch] Dubbel-tik onder - scroll omlaag");
                            }
                            touchHandled = true;
                        }
                        // Sla deze tik op voor volgende dubbel-tik detectie
                        lastTapTime = now;
                        lastTapY = touchY;
                    }
                }
                // Verticale swipes worden NIET verwerkt in normale schermen
                // (alleen horizontale swipes voor scherm navigatie)
            }
            // Reset touching state
            isTouching = false;
        }
    } else {
        // Geen nieuwe interrupt - rapporteer laatste bekende state
        // Dit is nodig omdat LVGL continu pollt, ook tussen interrupts
        data->state = isTouching ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;

        // Als we nog steeds touchen, rapporteer laatste bekende positie
        if (isTouching) {
            data->point.x = touchX;
            data->point.y = touchY;
        }
    }
}

// ============================================================================
// FUNCTIE: connectToWiFi
// ============================================================================
/*
 * Maakt verbinding met het geconfigureerde WiFi netwerk.
 * Credentials komen uit wifi_config.h (WIFI_SSID, WIFI_PASSWORD)
 *
 * Werking:
 * --------
 * 1. Configureer ESP32 als WiFi Station (client mode)
 * 2. Start verbindingspoging met opgegeven SSID en wachtwoord
 * 3. Wacht tot verbinding gemaakt is (max 10 seconden)
 * 4. Rapporteer resultaat via Serial
 *
 * WiFi Modes:
 * - WIFI_STA: Station mode - verbindt met bestaand netwerk
 * - WIFI_AP: Access Point mode - maakt eigen netwerk
 * - WIFI_AP_STA: Beide tegelijk
 *
 * Timeout:
 * - 20 pogingen x 500ms = 10 seconden maximale wachttijd
 * - Dit voorkomt dat de startup vastloopt bij WiFi problemen
 *
 * Na succesvolle verbinding:
 * - WiFi.status() == WL_CONNECTED
 * - WiFi.localIP() geeft het toegewezen IP adres
 * - Internet toegang beschikbaar voor NTP
 */
void connectToWiFi() {
    // Start feedback naar Serial monitor
    Serial.print("[WiFi] Verbinden met netwerk...");

    // Configureer als WiFi Station (client)
    // Dit is nodig voordat we verbinden
    WiFi.mode(WIFI_STA);

    // Start verbinding met SSID en wachtwoord uit wifi_config.h
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Wacht op verbinding met timeout
    uint8_t retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) {
        delay(500);           // Wacht 500ms
        Serial.print(".");    // Progress indicator
        retries++;            // Tel pogingen
    }

    // Rapporteer resultaat
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(" OK");  // Verbinding gelukt!

        // Toon toegewezen IP adres
        Serial.print("[WiFi] IP adres: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println(" MISLUKT");  // Verbinding niet gelukt
        // Programma gaat door zonder WiFi - tijd sync zal niet werken
    }
}

// ============================================================================
// FUNCTIE: initTime
// ============================================================================
/*
 * Initialiseert tijd synchronisatie via NTP (Network Time Protocol).
 * Vereist een actieve WiFi verbinding.
 *
 * NTP Werking:
 * -----------
 * 1. ESP32 stuurt request naar NTP server
 * 2. Server antwoordt met accurate UTC tijd
 * 3. ESP32 past timezone offset toe
 * 4. Lokale tijd is nu gesynchroniseerd
 *
 * Configuratie (uit wifi_config.h):
 * - NTP_SERVER: bijv. "pool.ntp.org" (gedistribueerde NTP pool)
 * - TZ_INFO: Timezone string voor Europa/Brussel
 *
 * Timezone String Format:
 * "CET-1CEST,M3.5.0,M10.5.0/3" betekent:
 * - CET: Standaard tijdzone naam
 * - -1: UTC+1 (in TZ format is - eigenlijk +)
 * - CEST: Zomertijd naam
 * - M3.5.0: Zomertijd start: maand 3, week 5, dag 0 (laatste zondag maart)
 * - M10.5.0/3: Wintertijd start: maand 10, week 5, dag 0, om 3:00
 *
 * Error Handling:
 * - Max 10 pogingen om tijd te krijgen
 * - Bij falen: ntpSynced blijft false
 * - Achtergrond sync probeert elke 60 seconden opnieuw
 */
void initTime() {
    Serial.println("[NTP] Tijd synchroniseren...");

    // Configureer tijdzone en NTP server
    // Deze functie start de achtergrond NTP sync
    configTzTime(TZ_INFO, NTP_SERVER);

    // Wacht op succesvolle synchronisatie
    struct tm timeinfo;  // Struct voor tijd data
    int retries = 0;

    // Probeer max 10 keer om lokale tijd te krijgen
    // getLocalTime() returned false als tijd nog niet gesync'd is
    while (!getLocalTime(&timeinfo, 500) && retries < 10) {
        retries++;  // Tel pogingen
    }

    // Bepaal of sync succesvol was
    ntpSynced = (retries < 10);  // true als we binnen 10 pogingen sync hadden

    // Sla tijdstip van poging op
    lastNtpAttempt = millis();

    // Rapporteer resultaat
    if (ntpSynced) {
        // Toon gesynchroniseerde tijd
        Serial.printf("[NTP] Tijd gesynchroniseerd: %02d:%02d:%02d\n",
            timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
        Serial.println("[NTP] Sync mislukt - zal later opnieuw proberen");
    }
}

// ============================================================================
// FUNCTIE: checkNtpSync
// ============================================================================
/*
 * Controleert periodiek of NTP synchronisatie nodig is.
 * Probeert opnieuw te synchroniseren als de vorige poging mislukt was.
 *
 * Wordt elke seconde aangeroepen vanuit de main loop.
 *
 * Werking:
 * --------
 * 1. Skip als al gesynchroniseerd (ntpSynced == true)
 * 2. Skip als geen WiFi verbinding
 * 3. Skip als minder dan 60 seconden sinds laatste poging
 * 4. Probeer tijd op te halen
 * 5. Bij succes: zet ntpSynced op true
 *
 * Waarom 60 seconden interval?
 * - Voorkomt overbelasten van NTP servers
 * - NTP pool servers hebben usage policies
 * - Eén succesvolle sync is voldoende (ESP32 houdt tijd bij)
 */
void checkNtpSync() {
    // Skip als tijd al gesynchroniseerd is
    if (ntpSynced) return;

    // Skip als geen WiFi verbinding (NTP vereist internet)
    if (WiFi.status() != WL_CONNECTED) return;

    // Skip als minder dan 60 seconden sinds laatste poging
    // Dit voorkomt te frequente NTP requests
    if (millis() - lastNtpAttempt < 60000) return;

    // Update tijdstip van deze poging
    lastNtpAttempt = millis();

    // Probeer lokale tijd op te halen
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 100)) {
        // Succes! Markeer als gesynchroniseerd
        ntpSynced = true;
        Serial.println("[NTP] Achtergrond synchronisatie geslaagd!");
    }
    // Bij falen: niets doen, volgende poging over 60 seconden
}

// ============================================================================
// FUNCTIE: updateTimeDisplay
// ============================================================================
/*
 * Werkt de tijd labels op het klokscherm bij.
 * Wordt elke seconde aangeroepen vanuit de main loop.
 *
 * Parameters:
 *   hours   - Uren in 24-uurs formaat (0-23)
 *   minutes - Minuten (0-59)
 *   seconds - Seconden (0-59)
 *
 * Optimalisatie:
 * -------------
 * De functie gebruikt statische buffers en vergelijkt nieuwe waarden
 * met vorige waarden. Alleen bij wijziging wordt het LVGL label
 * bijgewerkt. Dit voorkomt:
 * - Onnodige string operaties
 * - Onnodige LVGL redraws
 * - Flikkeren van de display
 *
 * Label Updates:
 * - ui_time_label: Toont "HH:MM" (bijv. "14:30")
 * - ui_seconds_label: Toont "SS" (bijv. "45")
 *
 * Format:
 * - %02d zorgt voor leading zero (9 wordt "09")
 * - Dubbele punt als separator
 */
void updateTimeDisplay(int hours, int minutes, int seconds) {
    // Statische buffers - behouden waarde tussen functie aanroepen
    // Dit maakt vergelijking met vorige waarde mogelijk
    static char timeBuf[8] = "";   // "HH:MM\0" = 6 chars, 8 voor veiligheid
    static char secBuf[4] = "";    // "SS\0" = 3 chars, 4 voor veiligheid

    // Tijdelijke buffers voor nieuwe waarden
    char newTime[8];
    char newSec[4];

    // Formatteer nieuwe tijd string
    // snprintf is veilig: stopt bij buffer grootte
    snprintf(newTime, sizeof(newTime), "%02d:%02d", hours, minutes);

    // Formatteer nieuwe seconden string
    snprintf(newSec, sizeof(newSec), "%02d", seconds);

    // Update tijd label ALLEEN als waarde gewijzigd is
    if (strcmp(newTime, timeBuf) != 0) {
        // Kopieer nieuwe waarde naar statische buffer
        strcpy(timeBuf, newTime);

        // Update LVGL label
        lv_label_set_text(ui_time_label, timeBuf);

        // Forceer volledige invalidatie van het label
        // Dit zorgt ervoor dat LVGL het label volledig hertekent
        lv_obj_invalidate(ui_time_label);
    }

    // Update seconden label ALLEEN als waarde gewijzigd is
    if (strcmp(newSec, secBuf) != 0) {
        // Kopieer nieuwe waarde naar statische buffer
        strcpy(secBuf, newSec);

        // Update LVGL label
        lv_label_set_text(ui_seconds_label, secBuf);

        // Forceer volledige invalidatie van het label
        lv_obj_invalidate(ui_seconds_label);
    }
}

// ============================================================================
// FUNCTIE: updateDateDisplay
// ============================================================================
/*
 * Werkt de datum en dag labels op het klokscherm bij.
 * Wordt elke seconde aangeroepen (maar update alleen bij wijziging).
 *
 * Parameters:
 *   day     - Dag van de maand (1-31)
 *   month   - Maand (0-11, volgens C standaard tm_mon!)
 *   year    - Jaar (volledig getal, bijv. 2026)
 *   weekday - Dag van de week (0-6, 0=zondag volgens tm_wday)
 *
 * Display Format:
 * - Datum: "17 Januari 2026" (dag maandnaam jaar)
 * - Dag: "Vrijdag" (Nederlandse dagnaam)
 *
 * Optimalisatie:
 * - Zelfde patroon als updateTimeDisplay()
 * - Statische buffers voor vergelijking
 * - Update alleen bij wijziging
 *
 * Datum wijzigt normaal alleen om middernacht, dus deze
 * optimalisatie is zeer effectief (slechts 1 update per dag).
 */
void updateDateDisplay(int day, int month, int year, int weekday) {
    // Statische buffers voor vergelijking met vorige waarde
    static char dateBuf[32] = "";  // "17 September 2026\0" past ruim
    static char dayBuf[16] = "";   // "Donderdag\0" = 10 chars, 16 voor veiligheid

    // Tijdelijke buffers voor nieuwe waarden
    char newDate[32];
    char newDay[16];

    // Formatteer datum: "dag maandnaam jaar"
    // maandNamen[month] haalt Nederlandse naam uit array
    snprintf(newDate, sizeof(newDate), "%d %s %d", day, maandNamen[month], year);

    // Formatteer dagnaam
    // dagNamen[weekday] haalt Nederlandse naam uit array
    snprintf(newDay, sizeof(newDay), "%s", dagNamen[weekday]);

    // Update datum label alleen bij wijziging
    if (strcmp(newDate, dateBuf) != 0) {
        strcpy(dateBuf, newDate);
        lv_label_set_text(ui_date_label, dateBuf);
        lv_obj_invalidate(ui_date_label);
    }

    // Update dag label alleen bij wijziging
    if (strcmp(newDay, dayBuf) != 0) {
        strcpy(dayBuf, newDay);
        lv_label_set_text(ui_day_label, dayBuf);
        lv_obj_invalidate(ui_day_label);
    }
}

// ============================================================================
// FUNCTIE: updateBatteryLevel
// ============================================================================
/*
 * Werkt het batterij percentage label bij op het instellingenscherm.
 * De tekstkleur verandert gebaseerd op het batterijniveau.
 *
 * Parameters:
 *   percentage - Batterij niveau (0-100)
 *
 * Kleurcodering:
 * - Groen (0x00FF00): > 50% - Goed geladen
 * - Geel (0xFFFF00): 20-50% - Matig geladen
 * - Rood (0xFF0000): < 20% - Bijna leeg, laden aanbevolen
 *
 * Deze visuele feedback helpt de gebruiker snel de batterijstatus
 * te beoordelen zonder het percentage te hoeven lezen.
 *
 * Optimalisatie:
 * - Statische variabele lastBatt slaat vorige waarde op
 * - Skip update als percentage niet gewijzigd is
 * - Batterij percentage verandert langzaam, dus effectief
 */
void updateBatteryLevel(int percentage) {
    // Statische variabele om vorige waarde te onthouden
    // -1 als initiële waarde zorgt dat eerste update altijd gebeurt
    static int lastBatt = -1;

    // Skip als percentage niet gewijzigd
    if (percentage == lastBatt) return;

    // Sla nieuwe waarde op
    lastBatt = percentage;

    // Formatteer percentage string met % teken
    static char battBuf[8];  // "100%\0" = 5 chars
    snprintf(battBuf, sizeof(battBuf), "%d%%", percentage);

    // Update label tekst
    lv_label_set_text(ui_battery_value, battBuf);

    // Bepaal kleur gebaseerd op niveau
    uint32_t color;
    if (percentage > 50) {
        // Goed geladen - groene kleur
        color = 0x00FF00;  // RGB: 0, 255, 0
    } else if (percentage > 20) {
        // Matig geladen - gele kleur
        color = 0xFFFF00;  // RGB: 255, 255, 0
    } else {
        // Bijna leeg - rode kleur (waarschuwing)
        color = 0xFF0000;  // RGB: 255, 0, 0
    }

    // Pas tekstkleur toe op het label
    // Parameter 0 = standaard state (niet pressed, focused, etc.)
    lv_obj_set_style_text_color(ui_battery_value, lv_color_hex(color), 0);

    // Forceer hertekenen
    lv_obj_invalidate(ui_battery_value);
}

// ============================================================================
// FUNCTIE: updateWiFiStatus
// ============================================================================
/*
 * Werkt de WiFi status label bij op het instellingenscherm.
 * Toont "Verbonden" (blauw) of "Geen" (rood).
 *
 * Parameters:
 *   connected - true als WiFi verbonden is, false als niet verbonden
 *
 * Weergave:
 * - Verbonden: Tekst "Verbonden" in cyaan/blauw (0x00C8FF)
 * - Niet verbonden: Tekst "Geen" in rood (0xFF0000)
 *
 * Optimalisatie:
 * - Statische variabele lastState slaat vorige status op
 * - Skip update als status niet gewijzigd
 * - WiFi status verandert zelden, dus zeer effectief
 */
void updateWiFiStatus(bool connected) {
    // Statische variabele voor vorige status
    // false als default - eerste check zal altijd matchen bij verbonden
    static bool lastState = false;

    // Skip als status niet gewijzigd
    if (connected == lastState) return;

    // Sla nieuwe status op
    lastState = connected;

    // Update label gebaseerd op status
    if (connected) {
        // WiFi is verbonden
        lv_label_set_text(ui_wifi_status, "Verbonden");

        // Cyaan/blauw kleur voor positieve status
        lv_obj_set_style_text_color(ui_wifi_status, lv_color_hex(0x00C8FF), 0);
    } else {
        // WiFi is niet verbonden
        lv_label_set_text(ui_wifi_status, "Geen");

        // Rode kleur voor negatieve status
        lv_obj_set_style_text_color(ui_wifi_status, lv_color_hex(0xFF0000), 0);
    }

    // Forceer hertekenen
    lv_obj_invalidate(ui_wifi_status);
}

// ============================================================================
// FUNCTIE: nextScreen
// ============================================================================
/*
 * Navigeert naar het volgende scherm met een slide animatie naar links.
 * Wordt aangeroepen bij swipe naar links gesture.
 *
 * Scherm Volgorde (cyclisch):
 * Klok (0) -> Stappen (1) -> Instellingen (2) -> Bestanden (3) -> Klok (0) -> ...
 *
 * Animatie:
 * - Type: LV_SCR_LOAD_ANIM_MOVE_LEFT (schuift naar links)
 * - Duur: 300ms
 * - Delay: 0ms (start direct)
 * - Auto delete: false (behoud vorige scherm in geheugen)
 *
 * De animatie geeft visuele feedback dat de swipe gesture
 * herkend is en maakt de overgang vloeiend.
 */
void nextScreen() {
    // Registreer activiteit (wake up display indien nodig)
    registerActivity();

    // Bereken volgend scherm index (cyclisch: 0->1->2->3->0)
    // Modulo 4 zorgt voor wrap-around (4 schermen)
    currentScreen = (currentScreen + 1) % 4;

    // Laad het nieuwe scherm met animatie
    switch(currentScreen) {
        case 0:
            // Laad klokscherm
            lv_scr_load_anim(ui_ScreenClock, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
            break;
        case 1:
            // Laad stappenscherm
            lv_scr_load_anim(ui_ScreenSteps, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
            break;
        case 2:
            // Laad instellingenscherm
            lv_scr_load_anim(ui_ScreenSettings, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
            break;
        case 3:
            // Laad bestandenscherm - update file lijst
            populateFileList();
            lv_scr_load_anim(ui_ScreenFiles, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
            break;
    }

    // Debug output
    Serial.printf("[UI] Scherm gewisseld naar: %d\n", currentScreen);
}

// ============================================================================
// FUNCTIE: prevScreen
// ============================================================================
/*
 * Navigeert naar het vorige scherm met een slide animatie naar rechts.
 * Wordt aangeroepen bij swipe naar rechts gesture.
 *
 * Scherm Volgorde (cyclisch, omgekeerd):
 * Klok (0) <- Stappen (1) <- Instellingen (2) <- Bestanden (3) <- Klok (0) <- ...
 *
 * Berekening:
 * - (currentScreen + 3) % 4 geeft vorig scherm
 * - Equivalent aan (currentScreen - 1 + 4) % 4 maar zonder negatieve getallen
 * - 0 -> 3, 1 -> 0, 2 -> 1, 3 -> 2
 *
 * Animatie:
 * - Type: LV_SCR_LOAD_ANIM_MOVE_RIGHT (schuift naar rechts)
 * - Duur: 300ms
 * - Consistente ervaring met nextScreen() (tegengestelde richting)
 */
void prevScreen() {
    // Registreer activiteit (wake up display indien nodig)
    registerActivity();

    // Bereken vorig scherm index
    // +3 mod 4 is equivalent aan -1 maar vermijdt negatieve getallen
    currentScreen = (currentScreen + 3) % 4;

    // Laad het nieuwe scherm met animatie (naar rechts)
    switch(currentScreen) {
        case 0:
            // Laad klokscherm
            lv_scr_load_anim(ui_ScreenClock, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
            break;
        case 1:
            // Laad stappenscherm
            lv_scr_load_anim(ui_ScreenSteps, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
            break;
        case 2:
            // Laad instellingenscherm
            lv_scr_load_anim(ui_ScreenSettings, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
            break;
        case 3:
            // Laad bestandenscherm - update file lijst
            populateFileList();
            lv_scr_load_anim(ui_ScreenFiles, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
            break;
    }

    // Debug output
    Serial.printf("[UI] Scherm gewisseld naar: %d\n", currentScreen);
}

// ============================================================================
// FUNCTIE: initSDCard
// ============================================================================
/*
 * Initialiseert de MicroSD kaart via SPI interface.
 *
 * Returns:
 *   true  - SD kaart succesvol gemount
 *   false - Geen SD kaart gevonden of mount mislukt
 *
 * De SD kaart gebruikt een aparte SPI bus (HSPI) om conflicten
 * met het QSPI display te voorkomen.
 *
 * Pinnen (uit pin_config.h):
 * - SDMMC_CLK (2)  -> SPI Clock
 * - SDMMC_CMD (1)  -> MOSI (Master Out, Slave In)
 * - SDMMC_DATA (3) -> MISO (Master In, Slave Out)
 * - SDMMC_CS (17)  -> Chip Select
 */
bool initSDCard() {
    Serial.println("[SD] MicroSD kaart initialiseren...");

    // Configureer SPI bus voor SD kaart
    // HSPI gebruikt andere pinnen dan de standaard VSPI
    sdSPI.begin(SDMMC_CLK, SDMMC_DATA, SDMMC_CMD, SDMMC_CS);

    // Probeer SD kaart te mounten
    // Parameters: CS pin, SPI bus, frequentie (MHz), mount point, max files
    if (!SD.begin(SDMMC_CS, sdSPI, 20000000)) {
        Serial.println("[SD] WAARSCHUWING: Geen SD kaart gevonden");
        sdCardAvailable = false;
        return false;
    }

    // Controleer kaart type
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("[SD] WAARSCHUWING: Geen SD kaart type gedetecteerd");
        sdCardAvailable = false;
        return false;
    }

    // Print kaart type
    Serial.print("[SD] Kaart type: ");
    switch (cardType) {
        case CARD_MMC:  Serial.println("MMC");  break;
        case CARD_SD:   Serial.println("SDSC"); break;
        case CARD_SDHC: Serial.println("SDHC"); break;
        default:        Serial.println("Onbekend"); break;
    }

    // Haal kaart grootte op
    sdCardSize = SD.cardSize();
    Serial.printf("[SD] Kaart grootte: %llu MB\n", sdCardSize / (1024 * 1024));
    Serial.printf("[SD] Totale ruimte: %llu MB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("[SD] Gebruikte ruimte: %llu MB\n", SD.usedBytes() / (1024 * 1024));

    sdCardAvailable = true;
    Serial.println("[SD] MicroSD kaart OK");
    return true;
}

/*
 * Lijst bestanden in een directory op de SD kaart.
 * Recursief als levels > 0.
 */
void listSDFiles(const char *dirname, uint8_t levels) {
    if (!sdCardAvailable) {
        Serial.println("[SD] Geen SD kaart beschikbaar");
        return;
    }

    Serial.printf("[SD] Directory: %s\n", dirname);

    File root = SD.open(dirname);
    if (!root) {
        Serial.println("[SD] Kan directory niet openen");
        return;
    }
    if (!root.isDirectory()) {
        Serial.println("[SD] Geen directory");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            Serial.print("  [DIR]  ");
            Serial.println(file.name());
            if (levels > 0) {
                // Recursief subdirectories tonen
                String path = String(dirname) + "/" + file.name();
                listSDFiles(path.c_str(), levels - 1);
            }
        } else {
            Serial.print("  [FILE] ");
            Serial.print(file.name());
            Serial.print("  (");
            Serial.print(file.size());
            Serial.println(" bytes)");
        }
        file = root.openNextFile();
    }
    root.close();
}

// ============================================================================
// AFBEELDING WEERGAVE FUNCTIES
// ============================================================================

/*
 * TJpg_Decoder Callback Functie
 * -----------------------------
 * Deze functie wordt aangeroepen door TJpg_Decoder voor elk blok (MCU)
 * van de JPEG afbeelding. Wij tekenen het blok naar het display.
 *
 * Parameters:
 *   x, y   - Positie van het blok op het display
 *   w, h   - Breedte en hoogte van het blok (meestal 8x8 of 16x16)
 *   bitmap - Pointer naar de pixel data in RGB565 formaat
 *
 * Returns:
 *   true om door te gaan met decoderen, false om te stoppen
 */
bool jpegDrawCallback(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
    // Pas offset toe voor centrering
    x += imageXOffset;
    y += imageYOffset;

    // Check of blok binnen scherm valt
    if (x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT) return true;
    if (x + w <= 0 || y + h <= 0) return true;

    // Teken het blok naar het display
    gfx->draw16bitRGBBitmap(x, y, bitmap, w, h);

    return true;  // Ga door met decoderen
}

/*
 * Toont een JPEG afbeelding van de SD kaart op het display.
 *
 * Parameters:
 *   filename - Pad naar het JPEG bestand (bijv. "/foto.jpg")
 *   centerX  - true om horizontaal te centreren
 *   centerY  - true om verticaal te centreren
 *
 * Returns:
 *   true bij succes, false bij fout
 */
bool showJpegFromSD(const char *filename, bool centerX, bool centerY) {
    if (!sdCardAvailable) {
        Serial.println("[IMG] Geen SD kaart beschikbaar");
        return false;
    }

    Serial.printf("[IMG] JPEG laden: %s\n", filename);

    // Configureer TJpg_Decoder - eerst zonder schaling om dimensies te krijgen
    TJpgDec.setJpgScale(1);
    TJpgDec.setSwapBytes(false);  // Geen byte swap - Arduino_GFX verwacht native RGB565
    TJpgDec.setCallback(jpegDrawCallback);

    // Haal afbeelding dimensies op
    uint16_t imgWidth = 0, imgHeight = 0;
    JRESULT result = TJpgDec.getSdJpgSize(&imgWidth, &imgHeight, filename);
    if (result != JDR_OK) {
        Serial.printf("[IMG] JPEG fout code: %d\n", result);
        // TJpgDec error codes:
        // 1=Interrupted, 2=Input error, 3=Memory pool, 4=Memory stream,
        // 5=Parameter, 6=Data format, 7=Right format not supported, 8=JPEG standard not supported
        if (result == 8 || result == 7) {
            Serial.println("[IMG] FOUT: Progressive JPEG of CMYK niet ondersteund!");
            Serial.println("[IMG] Converteer naar Baseline JPEG (RGB)");
            Serial.println("[IMG] Tip: Open in Paint en sla opnieuw op als JPEG");
        } else if (result == 6) {
            Serial.println("[IMG] FOUT: Beschadigd of ongeldig JPEG bestand");
        } else if (result == 2) {
            Serial.println("[IMG] FOUT: Kan bestand niet lezen van SD kaart");
        }
        return false;
    }

    Serial.printf("[IMG] Originele grootte: %dx%d\n", imgWidth, imgHeight);

    // Bepaal optimale schaalfactor (1, 2, 4 of 8)
    // Kies de kleinste schaal die de afbeelding binnen het scherm past
    uint8_t scale = 1;
    uint16_t scaledWidth = imgWidth;
    uint16_t scaledHeight = imgHeight;

    while ((scaledWidth > SCREEN_WIDTH || scaledHeight > SCREEN_HEIGHT) && scale < 8) {
        scale *= 2;
        scaledWidth = imgWidth / scale;
        scaledHeight = imgHeight / scale;
    }

    // Pas schaling toe
    TJpgDec.setJpgScale(scale);
    Serial.printf("[IMG] Schaal: 1/%d -> %dx%d\n", scale, scaledWidth, scaledHeight);

    // Bereken offset voor centrering met geschaalde dimensies
    imageXOffset = centerX ? (SCREEN_WIDTH - scaledWidth) / 2 : 0;
    imageYOffset = centerY ? (SCREEN_HEIGHT - scaledHeight) / 2 : 0;

    // Zorg dat offset niet negatief is
    if (imageXOffset < 0) imageXOffset = 0;
    if (imageYOffset < 0) imageYOffset = 0;

    // Maak scherm zwart voor weergave
    gfx->fillScreen(BLACK);

    // Decodeer en toon de JPEG
    unsigned long startTime = millis();
    if (TJpgDec.drawSdJpg(0, 0, filename) != JDR_OK) {
        Serial.println("[IMG] JPEG decoderen mislukt");
        return false;
    }

    unsigned long elapsed = millis() - startTime;
    Serial.printf("[IMG] JPEG weergegeven in %lu ms\n", elapsed);

    return true;
}

/*
 * PNG Draw Callback Functie
 * -------------------------
 * Wordt aangeroepen voor elke regel van de PNG afbeelding.
 * Moet 1 returnen om door te gaan, 0 om te stoppen.
 */
// Teller voor PNG callback diagnostiek
static int pngCallbackCount = 0;

int pngDrawCallback(PNGDRAW *pDraw) {
    uint16_t lineBuffer[SCREEN_WIDTH];

    // Converteer PNG regel naar RGB565 (native little-endian voor ESP32)
    png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_LITTLE_ENDIAN, 0xffffffff);

    pngCallbackCount++;

    // PSRAM BUFFER MODUS: kopieer de regel naar de PSRAM buffer
    // Na het decoderen tekent drawPngAtXY() de hele buffer in één keer
    // met draw16bitRGBBitmap(). Dit is nodig omdat de CO5300 QSPI display
    // niet goed werkt met draw16bitRGBBitmap(h=1) voor enkele rijen.
    if (pngPsramBuffer && pDraw->y >= 0 && pDraw->y < pngBufferHeight) {
        int16_t copyW = min((int)pDraw->iWidth, (int)pngBufferWidth);
        memcpy(&pngPsramBuffer[pDraw->y * pngBufferWidth], lineBuffer,
               copyW * sizeof(uint16_t));
        return 1;
    }

    // DIRECTE MODUS (fallback als geen PSRAM buffer beschikbaar)
    int16_t y = pDraw->y + imageYOffset;
    if (y < 0 || y >= SCREEN_HEIGHT) return 1;

    int16_t x = imageXOffset;
    int16_t w = pDraw->iWidth;

    if (x < 0) {
        w += x;
        x = 0;
    }
    if (x + w > SCREEN_WIDTH) {
        w = SCREEN_WIDTH - x;
    }
    if (w <= 0) return 1;

    gfx->draw16bitRGBBitmap(x, y, lineBuffer, w, 1);

    return 1;
}

/*
 * Toont een PNG afbeelding van de SD kaart op het display.
 */
bool showPngFromSD(const char *filename, bool centerX, bool centerY) {
    if (!sdCardAvailable) {
        Serial.println("[IMG] Geen SD kaart beschikbaar");
        return false;
    }

    Serial.printf("[IMG] PNG laden: %s\n", filename);

    // Open PNG bestand
    int rc = png.open(filename, [](const char *fn, int32_t *size) -> void* {
        File *f = new File(SD.open(fn, FILE_READ));
        if (!f || !*f) return nullptr;
        *size = f->size();
        return f;
    }, [](void *handle) {
        File *f = (File*)handle;
        f->close();
        delete f;
    }, [](PNGFILE *pFile, uint8_t *buffer, int32_t length) -> int32_t {
        File *f = (File*)pFile->fHandle;
        return f->read(buffer, length);
    }, [](PNGFILE *pFile, int32_t position) -> int32_t {
        File *f = (File*)pFile->fHandle;
        return f->seek(position) ? position : 0;
    }, pngDrawCallback);

    if (rc != PNG_SUCCESS) {
        Serial.printf("[IMG] PNG openen mislukt: %d\n", rc);
        return false;
    }

    int16_t pngW = png.getWidth();
    int16_t pngH = png.getHeight();
    Serial.printf("[IMG] PNG grootte: %dx%d\n", pngW, pngH);

    // Bereken offset voor centrering
    imageXOffset = centerX ? (SCREEN_WIDTH - pngW) / 2 : 0;
    imageYOffset = centerY ? (SCREEN_HEIGHT - pngH) / 2 : 0;
    if (imageXOffset < 0) imageXOffset = 0;
    if (imageYOffset < 0) imageYOffset = 0;

    // Maak scherm zwart
    gfx->fillScreen(BLACK);

    // Alloceer PSRAM buffer voor hele PNG (zelfde als drawPngAtXY)
    uint32_t bufSize = (uint32_t)pngW * pngH * sizeof(uint16_t);
    pngPsramBuffer = (uint16_t*)heap_caps_malloc(bufSize, MALLOC_CAP_SPIRAM);
    pngBufferWidth = pngW;
    pngBufferHeight = pngH;
    pngCallbackCount = 0;

    if (pngPsramBuffer) {
        memset(pngPsramBuffer, 0, bufSize);
    }

    // Decodeer PNG
    unsigned long startTime = millis();
    rc = png.decode(NULL, 0);
    png.close();

    if (rc != PNG_SUCCESS) {
        Serial.printf("[IMG] PNG decoderen mislukt: %d\n", rc);
        if (pngPsramBuffer) {
            heap_caps_free(pngPsramBuffer);
            pngPsramBuffer = nullptr;
        }
        return false;
    }

    // Teken de hele buffer in één keer
    if (pngPsramBuffer) {
        gfx->draw16bitRGBBitmap(imageXOffset, imageYOffset, pngPsramBuffer, pngW, pngH);
        heap_caps_free(pngPsramBuffer);
        pngPsramBuffer = nullptr;
    }

    Serial.printf("[IMG] PNG weergegeven in %lu ms\n", millis() - startTime);
    return true;
}

/*
 * Tekent een PNG afbeelding van SD kaart op specifieke X,Y positie.
 * Maakt NIET het scherm leeg (geen fillScreen).
 * Gebruikt de bestaande pngDrawCallback met imageXOffset/imageYOffset.
 *
 * Parameters:
 *   sdPath: pad naar het PNG bestand op de SD kaart (bijv. "/images/button_play_75.png")
 *   x, y: linkerbovenhoek waar de PNG getekend wordt
 *
 * Returns:
 *   true als PNG succesvol geladen en getekend, false bij fout
 */
bool drawPngAtXY(const char* sdPath, int16_t x, int16_t y) {
    unsigned long t0 = millis();
    Serial.printf("[IMG] PNG op (%d,%d): %s\n", x, y, sdPath);

    // Reset callback teller
    pngCallbackCount = 0;

    // Stel offsets in (voor directe modus fallback)
    imageXOffset = x;
    imageYOffset = y;

    // Open het PNG bestand van SD kaart
    int rc = png.open(sdPath, [](const char *fn, int32_t *size) -> void* {
        File *f = new File(SD.open(fn, FILE_READ));
        if (!f || !*f) {
            if (f) delete f;
            return nullptr;
        }
        *size = f->size();
        return f;
    }, [](void *handle) {
        File *f = (File*)handle;
        f->close();
        delete f;
    }, [](PNGFILE *pFile, uint8_t *buffer, int32_t length) -> int32_t {
        File *f = (File*)pFile->fHandle;
        return f->read(buffer, length);
    }, [](PNGFILE *pFile, int32_t position) -> int32_t {
        File *f = (File*)pFile->fHandle;
        return f->seek(position) ? position : 0;
    }, pngDrawCallback);

    if (rc != PNG_SUCCESS) {
        Serial.printf("[IMG] PNG open MISLUKT: %d (%s)\n", rc, sdPath);
        return false;
    }

    // Haal PNG dimensies op
    int16_t pngW = png.getWidth();
    int16_t pngH = png.getHeight();
    Serial.printf("[IMG] PNG info: %dx%d, %d bpp, offset(%d,%d)\n",
                  pngW, pngH, png.getBpp(), x, y);

    // Alloceer PSRAM buffer voor de hele afbeelding
    // CO5300 QSPI display vereist grotere transfers dan h=1
    // Door alles te bufferen en in één keer te tekenen werkt het correct
    uint32_t bufSize = (uint32_t)pngW * pngH * sizeof(uint16_t);
    pngPsramBuffer = (uint16_t*)heap_caps_malloc(bufSize, MALLOC_CAP_SPIRAM);
    pngBufferWidth = pngW;
    pngBufferHeight = pngH;

    if (pngPsramBuffer) {
        // Initialiseer buffer op zwart
        memset(pngPsramBuffer, 0, bufSize);
        Serial.printf("[IMG] PSRAM buffer: %u bytes gealloceerd\n", bufSize);
    } else {
        Serial.println("[IMG] PSRAM buffer MISLUKT - fallback naar directe modus");
    }

    // Decodeer PNG - callback vult PSRAM buffer (of tekent direct als fallback)
    rc = png.decode(NULL, 0);
    png.close();

    if (rc != PNG_SUCCESS) {
        Serial.printf("[IMG] PNG decode MISLUKT: %d\n", rc);
        if (pngPsramBuffer) {
            heap_caps_free(pngPsramBuffer);
            pngPsramBuffer = nullptr;
        }
        return false;
    }

    // Als PSRAM buffer beschikbaar: teken de hele afbeelding in één keer
    if (pngPsramBuffer) {
        // Clip afbeelding aan schermgrenzen
        int16_t drawW = pngW;
        int16_t drawH = pngH;
        int16_t drawX = x;
        int16_t drawY = y;

        if (drawX + drawW > SCREEN_WIDTH) drawW = SCREEN_WIDTH - drawX;
        if (drawY + drawH > SCREEN_HEIGHT) drawH = SCREEN_HEIGHT - drawY;
        if (drawX < 0) { drawX = 0; }
        if (drawY < 0) { drawY = 0; }

        Serial.printf("[IMG] Buffer tekenen: (%d,%d) %dx%d\n", drawX, drawY, drawW, drawH);
        gfx->draw16bitRGBBitmap(drawX, drawY, pngPsramBuffer, pngW, drawH);

        heap_caps_free(pngPsramBuffer);
        pngPsramBuffer = nullptr;
    }

    Serial.printf("[IMG] PNG klaar: %d regels, %lu ms\n", pngCallbackCount, millis() - t0);
    return true;
}

/*
 * Toont een afbeelding van SD kaart (detecteert automatisch formaat).
 */
bool showImageFromSD(const char *filename, bool centerX, bool centerY) {
    String fn = String(filename);
    fn.toLowerCase();

    if (fn.endsWith(".jpg") || fn.endsWith(".jpeg")) {
        return showJpegFromSD(filename, centerX, centerY);
    }
    else if (fn.endsWith(".png")) {
        return showPngFromSD(filename, centerX, centerY);
    }
    else if (fn.endsWith(".gif")) {
        // GIF ondersteuning - toon eerste frame als statische afbeelding
        // TODO: Animatie ondersteuning toevoegen
        Serial.println("[IMG] GIF formaat - statisch frame");
        return showGifFromSD(filename, centerX, centerY);
    }
    else if (fn.endsWith(".bmp")) {
        // BMP ondersteuning
        Serial.println("[IMG] BMP formaat");
        return showBmpFromSD(filename, centerX, centerY);
    }
    else if (fn.endsWith(".pcx")) {
        // PCX formaat - niet direct ondersteund
        Serial.println("[IMG] PCX formaat niet ondersteund");
        gfx->setTextColor(WHITE);
        gfx->setTextSize(2);
        gfx->setCursor(50, 230);
        gfx->print("PCX niet ondersteund");
        return true;  // Return true zodat viewer actief blijft
    }
    else {
        Serial.println("[IMG] Onbekend afbeelding formaat");
        return false;
    }
}

/*
 * Toont een GIF afbeelding van SD kaart
 * TODO: Volledige GIF animatie ondersteuning toevoegen
 */
bool showGifFromSD(const char *filename, bool centerX, bool centerY) {
    Serial.printf("[IMG] GIF laden: %s\n", filename);

    // Gebruik Arduino_Canvas voor tekst rendering (CO5300 QSPI vereist dit)
    Arduino_Canvas *canvas = new Arduino_Canvas(SCREEN_WIDTH, SCREEN_HEIGHT, gfx, 0, 0);
    if (!canvas || !canvas->begin(GFX_SKIP_OUTPUT_BEGIN)) {
        Serial.println("[IMG] GIF canvas mislukt");
        if (canvas) delete canvas;
        return false;
    }

    canvas->fillScreen(BLACK);

    // GIF icoon
    canvas->setTextColor(GREEN);
    canvas->setTextSize(3);
    canvas->setCursor(SCREEN_WIDTH/2 - 40, 150);
    canvas->print("GIF");

    // Bestandsnaam
    canvas->setTextColor(WHITE);
    canvas->setTextSize(1);

    String fn = String(filename);
    int lastSlash = fn.lastIndexOf('/');
    if (lastSlash >= 0) fn = fn.substring(lastSlash + 1);

    int x = (SCREEN_WIDTH - fn.length() * 6) / 2;
    if (x < 10) x = 10;
    canvas->setCursor(x, 220);
    canvas->print(fn);

    // Status tekst
    canvas->setTextColor(YELLOW);
    canvas->setCursor(SCREEN_WIDTH/2 - 80, 280);
    canvas->print("Animatie niet ondersteund");

    canvas->setTextColor(DARKGREY);
    canvas->setCursor(SCREEN_WIDTH/2 - 100, 350);
    canvas->print("Swipe voor volgende/vorige");
    canvas->setCursor(SCREEN_WIDTH/2 - 80, 370);
    canvas->print("3 sec drukken = terug");

    // Stuur naar display
    canvas->flush();
    delete canvas;

    Serial.println("[IMG] GIF placeholder getoond");
    return true;
}

/*
 * Toont een BMP afbeelding van SD kaart
 */
bool showBmpFromSD(const char *filename, bool centerX, bool centerY) {
    Serial.printf("[IMG] BMP laden: %s\n", filename);

    File file = SD.open(filename, FILE_READ);
    if (!file) {
        Serial.println("[IMG] Kon BMP niet openen");
        return false;
    }

    // Lees BMP header
    uint8_t header[54];
    if (file.read(header, 54) != 54) {
        Serial.println("[IMG] Ongeldige BMP header");
        file.close();
        return false;
    }

    // Controleer BMP signature
    if (header[0] != 'B' || header[1] != 'M') {
        Serial.println("[IMG] Geen geldig BMP bestand");
        file.close();
        return false;
    }

    // Haal afmetingen uit header
    int32_t width = *(int32_t*)&header[18];
    int32_t height = *(int32_t*)&header[22];
    uint16_t bpp = *(uint16_t*)&header[28];
    uint32_t dataOffset = *(uint32_t*)&header[10];

    Serial.printf("[IMG] BMP: %dx%d, %d bpp\n", width, height, bpp);

    // Alleen 24-bit BMP ondersteund voor nu
    if (bpp != 24) {
        Serial.printf("[IMG] Alleen 24-bit BMP ondersteund, dit is %d-bit\n", bpp);
        gfx->setTextColor(WHITE);
        gfx->setCursor(50, 230);
        gfx->printf("BMP %d-bit niet ondersteund", bpp);
        file.close();
        return true;
    }

    // Bereken offset voor centrering
    imageXOffset = centerX ? (SCREEN_WIDTH - abs(width)) / 2 : 0;
    imageYOffset = centerY ? (SCREEN_HEIGHT - abs(height)) / 2 : 0;
    if (imageXOffset < 0) imageXOffset = 0;
    if (imageYOffset < 0) imageYOffset = 0;

    // BMP is bottom-up, dus we tekenen van onder naar boven
    int rowSize = ((width * 3 + 3) & ~3);  // Rijen zijn 4-byte aligned
    uint8_t* rowBuffer = (uint8_t*)malloc(rowSize);
    if (!rowBuffer) {
        Serial.println("[IMG] Geen geheugen voor BMP buffer");
        file.close();
        return false;
    }

    file.seek(dataOffset);

    bool bottomUp = (height > 0);
    height = abs(height);

    for (int y = 0; y < height; y++) {
        file.read(rowBuffer, rowSize);
        int destY = bottomUp ? (height - 1 - y + imageYOffset) : (y + imageYOffset);

        for (int x = 0; x < width; x++) {
            uint8_t b = rowBuffer[x * 3];
            uint8_t g = rowBuffer[x * 3 + 1];
            uint8_t r = rowBuffer[x * 3 + 2];
            uint16_t color = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
            gfx->drawPixel(x + imageXOffset, destY, color);
        }
    }

    free(rowBuffer);
    file.close();

    Serial.println("[IMG] BMP weergegeven");
    return true;
}

// ============================================================================
// HELPER: fillScreenBlackPSRAM
// ============================================================================
/*
 * Vult het hele scherm met zwart via een PSRAM buffer.
 *
 * WAAROM NIET gewoon gfx->fillScreen(BLACK)?
 * Het CO5300 QSPI display heeft een eigenaardigheid: na fillScreen()
 * (dat writeRepeat/writeColor gebruikt) worden kleine pixel operaties
 * zoals gfx->print() NIET zichtbaar. Maar na een grote
 * draw16bitRGBBitmap() aanroep werken ze WEL.
 *
 * Dit komt doordat het QSPI display grotere data transfers nodig heeft
 * om de interne framebuffer correct te initialiseren. draw16bitRGBBitmap()
 * stuurt alle pixels als een aaneengesloten blok via QSPI, terwijl
 * fillScreen() een ander commando-pad gebruikt.
 *
 * Deze functie alloceert een zwarte buffer in PSRAM (410x502x2 = ~400KB),
 * tekent die in één keer naar het display, en geeft het geheugen vrij.
 * Daarna werken gfx->print(), drawLine(), drawCircle() etc. correct.
 *
 * Geheugen: ~400KB tijdelijk uit PSRAM (8MB beschikbaar).
 */
void fillScreenBlackPSRAM() {
    uint32_t bufSize = (uint32_t)SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t);
    uint16_t* bgBuffer = (uint16_t*)heap_caps_malloc(bufSize, MALLOC_CAP_SPIRAM);
    if (bgBuffer) {
        memset(bgBuffer, 0, bufSize);  // 0x0000 = BLACK in RGB565
        gfx->draw16bitRGBBitmap(0, 0, bgBuffer, SCREEN_WIDTH, SCREEN_HEIGHT);
        heap_caps_free(bgBuffer);
        Serial.println("[GFX] Zwart scherm via PSRAM buffer");
    } else {
        // Fallback als PSRAM niet beschikbaar
        gfx->fillScreen(BLACK);
        Serial.println("[GFX] WAARSCHUWING: PSRAM buffer mislukt, fillScreen fallback");
    }
}

/*
 * Geeft het geheugen vrij van de opgeslagen tekstregels
 */
void freeTextLines() {
    if (textFileLines) {
        delete[] textFileLines;
        textFileLines = nullptr;
    }
    textTotalLines = 0;
    textScrollLine = 0;
    textScrollCol = 0;
    textMaxCol = 0;
    textCurrentFile = "";
    Serial.println("[TEXT] Tekstregels geheugen vrijgegeven");
}

/*
 * Rendert de huidige pagina van het tekstbestand naar het display.
 * Gebruikt Arduino_Canvas als offscreen framebuffer.
 *
 * Het CO5300 QSPI display vereist dat alle pixels via draw16bitRGBBitmap()
 * worden gestuurd. Arduino_Canvas rendert naar een PSRAM buffer en stuurt
 * die in één keer via canvas->flush().
 */
void renderTextPage() {
    if (!textFileLines || textTotalLines == 0) {
        Serial.println("[TEXT] Geen tekst om te renderen");
        return;
    }

    // Bereken zichtbare regels per pagina
    int contentHeight = SCREEN_HEIGHT - TEXT_HEADER_Y - TEXT_FOOTER_HEIGHT;
    textLinesPerPage = contentHeight / TEXT_LINE_HEIGHT;
    if (textLinesPerPage < 1) textLinesPerPage = 1;

    // Clamp scroll positie
    int maxScroll = textTotalLines - textLinesPerPage;
    if (maxScroll < 0) maxScroll = 0;
    if (textScrollLine > maxScroll) textScrollLine = maxScroll;
    if (textScrollLine < 0) textScrollLine = 0;

    // Maak offscreen canvas
    Arduino_Canvas *canvas = new Arduino_Canvas(SCREEN_WIDTH, SCREEN_HEIGHT, gfx, 0, 0);
    if (!canvas || !canvas->begin(GFX_SKIP_OUTPUT_BEGIN)) {
        Serial.println("[TEXT] Canvas mislukt bij renderTextPage");
        if (canvas) delete canvas;
        return;
    }

    // Zwarte achtergrond
    canvas->fillScreen(BLACK);

    // === HEADER: bestandsnaam ===
    canvas->setTextColor(CYAN);
    canvas->setTextSize(TEXT_FONT_SIZE);
    canvas->setCursor(8, 5);

    // Korte bestandsnaam (zonder pad)
    String fn = textCurrentFile;
    int lastSlash = fn.lastIndexOf('/');
    if (lastSlash >= 0) fn = fn.substring(lastSlash + 1);
    if (fn.length() > TEXT_CHARS_PER_LINE) fn = fn.substring(0, TEXT_CHARS_PER_LINE - 3) + "...";
    canvas->print(fn);

    // Horizontale lijn onder header
    canvas->drawFastHLine(5, TEXT_HEADER_Y - 5, SCREEN_WIDTH - 10, DARKGREY);

    // === CONTENT: tekstregels met horizontale scroll ===
    canvas->setTextSize(TEXT_FONT_SIZE);
    canvas->setTextColor(WHITE);

    int endLine = textScrollLine + textLinesPerPage;
    if (endLine > textTotalLines) endLine = textTotalLines;

    for (int i = textScrollLine; i < endLine; i++) {
        int screenLine = i - textScrollLine;
        canvas->setCursor(8, TEXT_HEADER_Y + screenLine * TEXT_LINE_HEIGHT);

        // Pas horizontale scroll toe: toon substring vanaf textScrollCol
        if (textScrollCol < (int)textFileLines[i].length()) {
            String visible = textFileLines[i].substring(textScrollCol);
            // Beperk tot zichtbare breedte
            if (visible.length() > TEXT_CHARS_PER_LINE) {
                visible = visible.substring(0, TEXT_CHARS_PER_LINE);
            }
            canvas->print(visible);
        }
        // Als textScrollCol voorbij de regel is, tonen we niets (lege regel)
    }

    // === FOOTER: pagina-informatie en scroll indicatoren ===
    canvas->drawFastHLine(5, SCREEN_HEIGHT - TEXT_FOOTER_HEIGHT, SCREEN_WIDTH - 10, DARKGREY);

    // Pagina info
    canvas->setTextSize(1);
    canvas->setTextColor(gfx->color565(150, 150, 150));

    char pageInfo[50];
    if (textScrollCol > 0) {
        snprintf(pageInfo, sizeof(pageInfo), "R %d-%d/%d  K %d+",
                 textScrollLine + 1, endLine, textTotalLines, textScrollCol + 1);
    } else {
        snprintf(pageInfo, sizeof(pageInfo), "Regels %d-%d van %d",
                 textScrollLine + 1, endLine, textTotalLines);
    }
    canvas->setCursor(8, SCREEN_HEIGHT - TEXT_FOOTER_HEIGHT + 6);
    canvas->print(pageInfo);

    // Scroll indicatoren rechts
    canvas->setTextSize(1);
    int indicatorX = SCREEN_WIDTH - 40;
    // Verticale pijlen
    if (textScrollLine > 0) {
        canvas->setTextColor(YELLOW);
        canvas->setCursor(indicatorX, SCREEN_HEIGHT - TEXT_FOOTER_HEIGHT + 4);
        canvas->print("^");
    }
    if (endLine < textTotalLines) {
        canvas->setTextColor(YELLOW);
        canvas->setCursor(indicatorX + 10, SCREEN_HEIGHT - TEXT_FOOTER_HEIGHT + 4);
        canvas->print("v");
    }
    // Horizontale pijlen
    if (textScrollCol > 0) {
        canvas->setTextColor(CYAN);
        canvas->setCursor(indicatorX + 22, SCREEN_HEIGHT - TEXT_FOOTER_HEIGHT + 4);
        canvas->print("<");
    }
    if (textScrollCol + TEXT_CHARS_PER_LINE < textMaxCol) {
        canvas->setTextColor(CYAN);
        canvas->setCursor(indicatorX + 32, SCREEN_HEIGHT - TEXT_FOOTER_HEIGHT + 4);
        canvas->print(">");
    }

    // Help tekst
    canvas->setTextColor(DARKGREY);
    canvas->setCursor(8, SCREEN_HEIGHT - 10);
    canvas->print("Swipe: scroll | 3s druk: terug");

    // Stuur naar display
    canvas->flush();
    delete canvas;

    Serial.printf("[TEXT] Pagina gerenderd: regels %d-%d van %d\n",
                  textScrollLine + 1, endLine, textTotalLines);
}

/*
 * Opent een tekstbestand, laadt alle regels in geheugen, en toont de eerste pagina.
 *
 * De regels worden opgeslagen in een dynamisch array (textFileLines) zodat we
 * snel kunnen scrollen zonder het bestand opnieuw te lezen.
 * Bij het sluiten van de viewer wordt freeTextLines() aangeroepen.
 */
bool showTextFromSD(const char *filename) {
    Serial.printf("[TEXT] Laden: %s\n", filename);

    // Ruim eventuele vorige tekst op
    freeTextLines();

    File file = SD.open(filename, FILE_READ);
    if (!file) {
        Serial.println("[TEXT] Kon bestand niet openen");
        return false;
    }

    // Sla bestandsnaam op voor later gebruik
    textCurrentFile = String(filename);

    // === STAP 1: Tel het aantal regels ===
    // Lees eerst alle regels en sla ze tijdelijk op
    // Maximaal TEXT_MAX_LINES regels (500)
    textFileLines = new String[TEXT_MAX_LINES];
    if (!textFileLines) {
        Serial.println("[TEXT] Geheugen allocatie mislukt voor regels");
        file.close();
        return false;
    }

    textTotalLines = 0;
    textMaxCol = 0;
    while (file.available() && textTotalLines < TEXT_MAX_LINES) {
        String line = file.readStringUntil('\n');
        // Verwijder alleen \r (carriage return), behoud spaties aan het begin
        line.replace("\r", "");

        // Beperk regelbreedte tot max opslag (maar NIET tot schermbreedte)
        // Zo kunnen we horizontaal scrollen door lange regels
        if (line.length() > TEXT_MAX_LINE_LEN) {
            line = line.substring(0, TEXT_MAX_LINE_LEN);
        }

        // Houd de langste regel bij voor horizontale scroll limieten
        if ((int)line.length() > textMaxCol) {
            textMaxCol = (int)line.length();
        }

        textFileLines[textTotalLines] = line;
        textTotalLines++;
    }

    file.close();
    Serial.printf("[TEXT] %d regels geladen, langste: %d karakters\n", textTotalLines, textMaxCol);

    // === STAP 2: Toon eerste pagina ===
    textScrollLine = 0;
    textScrollCol = 0;
    renderTextPage();

    return true;
}

/*
 * Tekent achtergrond voor de audio speler
 * Donkerblauwe gradiënt - duidelijk zichtbaar
 */
void drawMusicBackground() {
    Serial.println("[AUDIO UI] Drawing background...");

    // Effen donkerblauwe achtergrond voor nu (snel en zichtbaar)
    gfx->fillScreen(gfx->color565(20, 20, 50));

    Serial.println("[AUDIO UI] Background done");
}

/*
 * Tekent een eenvoudige maar duidelijke vinyl plaat
 */
void drawVinylRecord(int centerX, int centerY, int radius, bool isPlaying) {
    Serial.println("[AUDIO UI] Drawing vinyl record...");

    // Buitenste rand
    gfx->fillCircle(centerX, centerY, radius, gfx->color565(30, 30, 35));

    // Groeven (simpele cirkels)
    for (int r = radius - 5; r > 30; r -= 6) {
        gfx->drawCircle(centerX, centerY, r, gfx->color565(50, 50, 55));
    }

    // Rood label in het midden
    gfx->fillCircle(centerX, centerY, 28, gfx->color565(200, 50, 50));
    gfx->drawCircle(centerX, centerY, 28, gfx->color565(255, 100, 100));

    // Gat in het midden
    gfx->fillCircle(centerX, centerY, 5, gfx->color565(20, 20, 30));

    // Playing indicator
    if (isPlaying) {
        gfx->fillCircle(centerX + 12, centerY - 8, 4, gfx->color565(255, 255, 150));
    }

    Serial.println("[AUDIO UI] Vinyl done");
}

/*
 * Tekent een Play button (driehoek)
 * centerX, centerY: centrum van de button
 * size: grootte van de button
 * highlighted: true = actieve staat
 */
void drawPlayButton(int centerX, int centerY, int size, bool highlighted) {
    uint16_t bgColor = highlighted ? gfx->color565(0, 180, 80) : gfx->color565(0, 140, 60);
    uint16_t fgColor = WHITE;
    uint16_t shadowColor = gfx->color565(0, 80, 30);

    // Schaduw
    gfx->fillCircle(centerX + 2, centerY + 2, size, shadowColor);

    // Button achtergrond (cirkel)
    gfx->fillCircle(centerX, centerY, size, bgColor);

    // Rand
    gfx->drawCircle(centerX, centerY, size, gfx->color565(0, 200, 100));
    gfx->drawCircle(centerX, centerY, size - 1, gfx->color565(0, 220, 110));

    // Play driehoek (wijst naar rechts)
    int triSize = size * 0.6;
    int x1 = centerX - triSize/2 + 3;  // Links (iets naar rechts voor centrering)
    int y1 = centerY - triSize;         // Boven
    int x2 = centerX - triSize/2 + 3;  // Links
    int y2 = centerY + triSize;         // Onder
    int x3 = centerX + triSize;         // Rechts (punt)
    int y3 = centerY;                   // Midden

    gfx->fillTriangle(x1, y1, x2, y2, x3, y3, fgColor);
}

/*
 * Tekent een Pause button (twee verticale balken)
 * centerX, centerY: centrum van de button
 * size: grootte van de button
 * highlighted: true = actieve staat
 */
void drawPauseButton(int centerX, int centerY, int size, bool highlighted) {
    uint16_t bgColor = highlighted ? gfx->color565(220, 160, 0) : gfx->color565(180, 130, 0);
    uint16_t fgColor = WHITE;
    uint16_t shadowColor = gfx->color565(100, 70, 0);

    // Schaduw
    gfx->fillCircle(centerX + 2, centerY + 2, size, shadowColor);

    // Button achtergrond (cirkel)
    gfx->fillCircle(centerX, centerY, size, bgColor);

    // Rand
    gfx->drawCircle(centerX, centerY, size, gfx->color565(255, 200, 0));
    gfx->drawCircle(centerX, centerY, size - 1, gfx->color565(255, 220, 50));

    // Pause balken
    int barWidth = size / 3;
    int barHeight = size;
    int gap = barWidth / 2 + 2;

    // Linker balk
    gfx->fillRoundRect(centerX - gap - barWidth/2, centerY - barHeight/2,
                       barWidth, barHeight, 2, fgColor);
    // Rechter balk
    gfx->fillRoundRect(centerX + gap - barWidth/2, centerY - barHeight/2,
                       barWidth, barHeight, 2, fgColor);
}

/*
 * Tekent een duidelijk zichtbare volume balk
 * volume: 0-21 (audio library range)
 */
void drawVolumeBar(uint8_t volume) {
    Serial.printf("[AUDIO UI] Volume bar, vol=%d\n", volume);

    int barWidth = 260;
    int barHeight = 16;
    int barX = (SCREEN_WIDTH - barWidth) / 2;  // Gecentreerd (75)
    int barY = 450;  // Positie onder de knoppen

    // Achtergrond balk (semi-transparant donkergrijs)
    gfx->fillRoundRect(barX, barY, barWidth, barHeight, 8, gfx->color565(30, 30, 50));
    gfx->drawRoundRect(barX, barY, barWidth, barHeight, 8, gfx->color565(120, 120, 160));

    // Gevulde deel (helder groen/cyan gradient effect)
    int fillWidth = (barWidth - 4) * volume / 21;
    if (fillWidth > 0) {
        gfx->fillRoundRect(barX + 2, barY + 2, fillWidth, barHeight - 4, 6, gfx->color565(0, 220, 120));
    }

    // "-" links van balk (duidelijk zichtbaar)
    gfx->setTextColor(WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(barX - 25, barY - 1);
    gfx->print("-");

    // "+" rechts van balk
    gfx->setCursor(barX + barWidth + 10, barY - 1);
    gfx->print("+");

    // Volume percentage onder de balk
    gfx->setTextColor(gfx->color565(220, 220, 240));
    gfx->setTextSize(1);
    char volText[16];
    int percent = volume * 100 / 21;
    snprintf(volText, sizeof(volText), "Vol: %d%%", percent);
    int textWidth = strlen(volText) * 6;  // Size 1 = 6px per char
    gfx->setCursor((SCREEN_WIDTH - textWidth) / 2, barY + barHeight + 5);
    gfx->print(volText);

    Serial.printf("[AUDIO UI] Volume bar done (%d%%)\n", percent);
}

/*
 * Tekent de controle knoppen met PNG images van SD kaart.
 * Layout: 4 knoppen op een rij (prev, play/pause, next, repeat)
 * Elke knop is 75x75 pixels.
 *
 * Button layout constanten (ook gebruikt in handleAudioTap):
 *   BTN_Y      = 360   Y positie van knoppen
 *   BTN_SIZE   = 75    Grootte van elke knop
 *   BTN_GAP    = 15    Ruimte tussen knoppen
 *   BTN_STARTX = 32    X positie van eerste knop
 *   BTN_STEP   = 90    Afstand tussen knop startposities (75 + 15)
 *
 * isPlaying: true = toon pause knop, false = toon play knop
 */
void drawControlButtons(bool isPlaying) {
    Serial.println("[AUDIO UI] Knoppen laden van SD...");

    // Button layout constanten
    const int btnY = 360;
    const int btnStep = 90;  // 75px button + 15px gap
    const int startX = 32;

    // ===== VORIGE KNOP =====
    if (!drawPngAtXY("/images/button_prev_75.png", startX, btnY)) {
        // Fallback: programmatische knop
        Serial.println("[AUDIO UI] FALLBACK: prev knop programmatisch");
        gfx->fillCircle(startX + 37, btnY + 37, 30, gfx->color565(80, 80, 120));
        gfx->drawCircle(startX + 37, btnY + 37, 30, WHITE);
    }

    // ===== PLAY OF PAUSE KNOP =====
    const char* playPauseImg = isPlaying ? "/images/button_pause_75.png" : "/images/button_play_75.png";
    if (!drawPngAtXY(playPauseImg, startX + btnStep, btnY)) {
        // Fallback: programmatische knop
        Serial.println("[AUDIO UI] FALLBACK: play/pause knop programmatisch");
        int cx = startX + btnStep + 37;
        int cy = btnY + 37;
        gfx->fillCircle(cx, cy, 30, isPlaying ? gfx->color565(255, 180, 0) : gfx->color565(0, 200, 0));
        gfx->drawCircle(cx, cy, 30, WHITE);
    }

    // ===== VOLGENDE KNOP =====
    if (!drawPngAtXY("/images/button_next_75.png", startX + 2 * btnStep, btnY)) {
        // Fallback: programmatische knop
        Serial.println("[AUDIO UI] FALLBACK: next knop programmatisch");
        gfx->fillCircle(startX + 2 * btnStep + 37, btnY + 37, 30, gfx->color565(80, 80, 120));
        gfx->drawCircle(startX + 2 * btnStep + 37, btnY + 37, 30, WHITE);
    }

    // ===== HERHAAL KNOP =====
    if (!drawPngAtXY("/images/button_repeat_75.png", startX + 3 * btnStep, btnY)) {
        // Fallback: programmatische knop
        Serial.println("[AUDIO UI] FALLBACK: repeat knop programmatisch");
        gfx->fillCircle(startX + 3 * btnStep + 37, btnY + 37, 30, gfx->color565(80, 80, 120));
        gfx->drawCircle(startX + 3 * btnStep + 37, btnY + 37, 30, WHITE);
    }

    Serial.println("[AUDIO UI] Knoppen klaar");
}

/*
 * Toont de complete audio player interface
 */
void showAudioPlayer(const char *filename) {
    unsigned long startMs = millis();
    Serial.println("[AUDIO] ========== PLAYER START ==========");
    Serial.printf("[AUDIO] Bestand: %s\n", filename);
    Serial.printf("[AUDIO] audioAvailable: %d, audioPlaying: %d\n", audioAvailable, audioPlaying);

    // Registreer activiteit om display dimming te voorkomen tijdens laden van PNGs
    registerActivity();

    // === STAP 1: Achtergrond afbeelding van SD kaart ===
    Serial.printf("[AUDIO] STAP 1: Achtergrond laden... (%lu ms)\n", millis() - startMs);
    bool bgLoaded = drawPngAtXY("/images/BG_Watch_Audio.png", 0, 0);
    if (!bgLoaded) {
        // Fallback: paarse achtergrond als PNG niet gevonden
        Serial.println("[AUDIO] PNG MISLUKT - fallback achtergrond");
        gfx->fillScreen(gfx->color565(60, 20, 80));
    }
    Serial.printf("[AUDIO] Achtergrond klaar (%lu ms)\n", millis() - startMs);

    // Registreer activiteit opnieuw (PNG laden kan lang duren)
    registerActivity();

    // === STAP 2: Song titel bovenaan ===
    String fn = String(filename);
    int lastSlash = fn.lastIndexOf('/');
    if (lastSlash >= 0) fn = fn.substring(lastSlash + 1);
    int dotPos = fn.lastIndexOf('.');
    if (dotPos > 0) fn = fn.substring(0, dotPos);
    if (fn.length() > 25) fn = fn.substring(0, 22) + "...";

    // Titel met schaduw effect voor leesbaarheid over achtergrond
    gfx->setTextSize(2);
    int textWidth = fn.length() * 12;
    int x = (SCREEN_WIDTH - textWidth) / 2;
    if (x < 10) x = 10;
    // Schaduw (zwart, 1px offset)
    gfx->setTextColor(BLACK);
    gfx->setCursor(x + 1, 21);
    gfx->print(fn);
    // Witte titel
    gfx->setTextColor(WHITE);
    gfx->setCursor(x, 20);
    gfx->print(fn);
    Serial.printf("[AUDIO] Titel: %s @ x=%d\n", fn.c_str(), x);

    // === STAP 3: "NOW PLAYING" subtitel ===
    gfx->setTextColor(gfx->color565(200, 200, 230));
    gfx->setTextSize(1);
    int nowPlayingX = (SCREEN_WIDTH - 11 * 6) / 2;  // 11 chars * 6px
    gfx->setCursor(nowPlayingX, 48);
    gfx->print("NOW PLAYING");

    // === STAP 4: Control knoppen (PNG images van SD kaart) ===
    Serial.printf("[AUDIO] STAP 4: Knoppen laden... (%lu ms)\n", millis() - startMs);
    drawControlButtons(true);
    Serial.printf("[AUDIO] Knoppen klaar (%lu ms)\n", millis() - startMs);

    // Registreer activiteit opnieuw na knoppen laden
    registerActivity();

    // === STAP 5: Volume balk ===
    Serial.printf("[AUDIO] STAP 5: Volume balk... (%lu ms)\n", millis() - startMs);
    drawVolumeBar(audioVolume);

    // === STAP 6: Help tekst onderaan ===
    gfx->setTextColor(gfx->color565(200, 200, 220));
    gfx->setTextSize(1);
    gfx->setCursor(45, SCREEN_HEIGHT - 18);
    gfx->print("Tap: bediening  Swipe: vol  3s: terug");

    Serial.printf("[AUDIO] ========== UI COMPLEET (%lu ms) ==========\n", millis() - startMs);

    // === STAP 7: Start audio playback ===
    if (audioAvailable) {
        Serial.printf("[AUDIO] STAP 7: Audio starten... (%lu ms)\n", millis() - startMs);
        Serial.printf("[AUDIO] Heap vrij: %d bytes, PSRAM vrij: %d bytes\n",
                      ESP.getFreeHeap(), ESP.getFreePsram());
        startAudioPlayback(filename);
        Serial.printf("[AUDIO] Na startAudioPlayback: audioPlaying=%d (%lu ms)\n",
                      audioPlaying, millis() - startMs);
    } else {
        Serial.println("[AUDIO] WAARSCHUWING: Audio NIET beschikbaar!");
        gfx->setTextColor(RED);
        gfx->setTextSize(2);
        gfx->setCursor(50, 250);
        gfx->print("Audio niet beschikbaar!");
    }
}


// ============================================================================
// FILE BROWSER FUNCTIES
// ============================================================================

/*
 * Geselecteerd bestandspad voor weergave
 * Wordt gezet bij file selectie en gebruikt door de viewer
 */
String selectedFilePath = "";

/*
 * Bepaalt het FileType van een bestand op basis van extensie
 */
FileType getFileTypeEnum(const char* filename) {
    String fn = String(filename);
    fn.toLowerCase();

    // Afbeeldingen: JPG, JPEG, PNG, GIF, BMP, PCX
    if (fn.endsWith(".jpg") || fn.endsWith(".jpeg") || fn.endsWith(".png") ||
        fn.endsWith(".gif") || fn.endsWith(".bmp") || fn.endsWith(".pcx")) {
        return FILE_TYPE_IMAGE;
    }
    // Tekst bestanden: TXT, LOG, CSV
    else if (fn.endsWith(".txt") || fn.endsWith(".log") || fn.endsWith(".csv")) {
        return FILE_TYPE_TEXT;
    }
    // Audio bestanden: MP3, WAV, OGG, FLAC
    else if (fn.endsWith(".mp3") || fn.endsWith(".wav") || fn.endsWith(".ogg") || fn.endsWith(".flac")) {
        return FILE_TYPE_AUDIO;
    }
    // Video bestanden worden niet meer ondersteund (verwijderd)
    return FILE_TYPE_NONE;
}

/*
 * Bouwt een lijst van bestanden van hetzelfde type
 * Wordt aangeroepen bij openen van een bestand
 */
void buildFileListByType(FileType targetType) {
    fileListCount = 0;
    currentFileIndex = 0;

    // Open de huidige directory (niet altijd root)
    File dir = SD.open(currentDirectory.c_str());
    if (!dir) {
        Serial.printf("[Viewer] Kon directory niet openen: %s\n", currentDirectory.c_str());
        return;
    }

    // Doorloop alle bestanden in de huidige directory
    File file = dir.openNextFile();
    while (file && fileListCount < 64) {
        if (!file.isDirectory()) {
            const char* fname = file.name();

            // Bouw volledig pad
            String fullPath = currentDirectory;
            if (!fullPath.endsWith("/")) fullPath += "/";
            fullPath += String(fname);

            // Check of dit bestand van het gewenste type is
            if (getFileTypeEnum(fname) == targetType) {
                fileList[fileListCount] = fullPath;

                // Check of dit het huidige geselecteerde bestand is
                if (fullPath == selectedFilePath) {
                    currentFileIndex = fileListCount;
                }

                fileListCount++;
                Serial.printf("[Viewer] Bestand toegevoegd: %s (index %d)\n", fullPath.c_str(), fileListCount - 1);
            }
        }
        file = dir.openNextFile();
    }

    dir.close();
    Serial.printf("[Viewer] %d bestanden van type %d gevonden in %s, huidige index: %d\n",
                  fileListCount, targetType, currentDirectory.c_str(), currentFileIndex);
}

/*
 * Verlaat de viewer en ga terug naar het bestandsscherm
 */
void exitFileViewer() {
    Serial.println("[Viewer] Verlaten viewer mode");

    // Stop eventuele audio playback
    stopAudioPlayback();

    // Geef tekst viewer geheugen vrij
    freeTextLines();

    // Reset ALLE touch state variabelen
    // Dit is kritiek om te voorkomen dat oude touch data swipes blokkeert
    fileViewerActive = false;
    currentViewerType = FILE_TYPE_NONE;
    viewerIsTouching = false;
    fileListCount = 0;
    currentFileIndex = 0;

    // Reset ook de algemene touch state (voor LVGL schermen)
    isTouching = false;
    touchHandled = false;
    touchStartX = 0;
    touchStartY = 0;
    lastTapTime = 0;

    // Wis eventuele pending touch interrupt
    FT3168->IIC_Interrupt_Flag = false;

    // Laad het bestandsscherm in LVGL
    lv_scr_load(ui_ScreenFiles);

    // KRITIEK: Forceer LVGL om het scherm DIRECT te renderen
    // Anders blijft ui_ScreenViewer zichtbaar totdat de volgende lv_timer_handler()
    lv_timer_handler();

    // Herlaad de bestandenlijst (ververst de weergave)
    populateFileList();

    Serial.println("[Viewer] Terug in bestanden scherm");
}

/*
 * handleViewerTouch
 * -----------------
 * Verwerkt touch input wanneer de file viewer actief is.
 *
 * EENVOUDIGE AANPAK: Negeer ALLE touch events voor 2 seconden na openen.
 * Dit voorkomt dat de originele touch of ghost touches worden verwerkt.
 *
 * Touch acties in viewer (na 2 sec wachttijd):
 * - Swipe omhoog: volgende bestand
 * - Swipe omlaag: vorige bestand
 * - Lang drukken (>3 sec): terug naar bestanden
 */
void handleViewerTouch() {
    // Alleen uitvoeren als viewer actief is
    if (!fileViewerActive) return;

    unsigned long currentTime = millis();
    unsigned long timeSinceActivation = currentTime - viewerActivatedTime;

    // =======================================================================
    // FASE 0: ABSOLUTE BLOKKADE - Eerste 2 seconden NA ACTIVATIE
    // =======================================================================
    // Dit is de meest betrouwbare manier om te voorkomen dat de touch van
    // de bestandsselectie wordt verwerkt als een swipe
    if (timeSinceActivation < 2000) {
        // Wis ALLE touch data zonder te verwerken
        if (FT3168->IIC_Interrupt_Flag) {
            FT3168->IIC_Interrupt_Flag = false;
            FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);
        }
        viewerIsTouching = false;
        viewerInitialReleaseDetected = false;

        // Debug elke 500ms
        static unsigned long lastBlockDebug = 0;
        if (currentTime - lastBlockDebug > 500) {
            Serial.printf("[Viewer] BLOKKADE: nog %lu ms\n", 2000 - timeSinceActivation);
            lastBlockDebug = currentTime;
        }
        return;
    }

    // =======================================================================
    // FASE 1: WACHT OP RELEASE - Na blokkade, wacht tot vinger is opgetild
    // =======================================================================
    if (!viewerInitialReleaseDetected) {
        if (FT3168->IIC_Interrupt_Flag) {
            FT3168->IIC_Interrupt_Flag = false;
            uint8_t touchPoints = FT3168->IIC_Read_Device_Value(
                FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

            if (touchPoints == 0) {
                viewerInitialReleaseDetected = true;
                viewerReleaseTime = currentTime;
                Serial.println("[Viewer] Release gedetecteerd - wacht 500ms debounce");
            } else {
                Serial.printf("[Viewer] Wacht op release, touchPoints=%d\n", touchPoints);
            }
        }
        viewerIsTouching = false;
        return;
    }

    // =======================================================================
    // FASE 2: DEBOUNCE - Wacht 500ms na release
    // =======================================================================
    if (currentTime - viewerReleaseTime < 500) {
        if (FT3168->IIC_Interrupt_Flag) {
            FT3168->IIC_Interrupt_Flag = false;
            FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);
        }
        viewerIsTouching = false;
        return;
    }

    // =======================================================================
    // FASE 3: NORMALE TOUCH VERWERKING
    // =======================================================================
    if (FT3168->IIC_Interrupt_Flag) {
        FT3168->IIC_Interrupt_Flag = false;

        uint8_t touchPoints = FT3168->IIC_Read_Device_Value(
            FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

        if (touchPoints > 0) {
            int16_t x = FT3168->IIC_Read_Device_Value(
                FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
            int16_t y = FT3168->IIC_Read_Device_Value(
                FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

            registerActivity();

            // Bij nieuwe touch, sla startpositie op
            if (!viewerIsTouching) {
                viewerIsTouching = true;
                viewerTouchStartTime = currentTime;
                viewerTouchStartX = x;
                viewerTouchStartY = y;
            }

            touchX = x;
            touchY = y;

            // Long press check (3 seconden)
            if (viewerIsTouching && (currentTime - viewerTouchStartTime >= LONG_PRESS_TIME)) {
                Serial.println("[Viewer] Long press - terug naar bestanden");
                viewerIsTouching = false;
                exitFileViewer();
                return;
            }

        } else {
            // Vinger losgelaten - check voor tap of swipe
            if (viewerIsTouching) {
                unsigned long touchDuration = currentTime - viewerTouchStartTime;
                int16_t deltaX = touchX - viewerTouchStartX;
                int16_t deltaY = touchY - viewerTouchStartY;
                bool isSmallMovement = (abs(deltaX) < 25 && abs(deltaY) < 25);

                // === TAP DETECTIE: korte touch met weinig beweging ===
                // Een tap duurt minder dan 500ms en heeft minder dan 25px beweging
                if (touchDuration < 500 && isSmallMovement) {
                    Serial.printf("[Viewer] TAP gedetecteerd op (%d, %d) duur=%lu ms\n",
                                  touchX, touchY, touchDuration);

                    // Audio player knoppen verwerken bij tap
                    if (currentViewerType == FILE_TYPE_AUDIO) {
                        handleAudioTap(touchX, touchY);
                    }
                }
                // === SWIPE DETECTIE: langere touch met grotere beweging ===
                else if (touchDuration >= 200 && touchDuration < LONG_PRESS_TIME) {
                    // --- TEKST VIEWER: swipe scrollt door het bestand ---
                    if (currentViewerType == FILE_TYPE_TEXT && abs(deltaY) > SWIPE_THRESHOLD && abs(deltaY) > abs(deltaX)) {
                        // Bereken scroll stap: half scherm per swipe
                        int scrollStep = textLinesPerPage / 2;
                        if (scrollStep < 3) scrollStep = 3;

                        if (deltaY < 0) {
                            // Swipe omhoog = scroll naar beneden (meer tekst zien)
                            int maxScroll = textTotalLines - textLinesPerPage;
                            if (maxScroll < 0) maxScroll = 0;
                            textScrollLine += scrollStep;
                            if (textScrollLine > maxScroll) textScrollLine = maxScroll;
                            Serial.printf("[Viewer] Tekst scroll omlaag: regel %d\n", textScrollLine);
                            renderTextPage();
                        } else {
                            // Swipe omlaag = scroll naar boven (terug)
                            textScrollLine -= scrollStep;
                            if (textScrollLine < 0) textScrollLine = 0;
                            Serial.printf("[Viewer] Tekst scroll omhoog: regel %d\n", textScrollLine);
                            renderTextPage();
                        }
                    }
                    // --- TEKST VIEWER: horizontaal scrollen ---
                    else if (currentViewerType == FILE_TYPE_TEXT && abs(deltaX) > SWIPE_THRESHOLD && abs(deltaX) > abs(deltaY)) {
                        if (deltaX < 0) {
                            // Swipe naar links = scroll naar rechts (meer tekst zien)
                            textScrollCol += TEXT_H_SCROLL_STEP;
                            int maxCol = textMaxCol - TEXT_CHARS_PER_LINE;
                            if (maxCol < 0) maxCol = 0;
                            if (textScrollCol > maxCol) textScrollCol = maxCol;
                            Serial.printf("[Viewer] Tekst scroll rechts: kolom %d\n", textScrollCol);
                            renderTextPage();
                        } else {
                            // Swipe naar rechts = scroll naar links (terug)
                            textScrollCol -= TEXT_H_SCROLL_STEP;
                            if (textScrollCol < 0) textScrollCol = 0;
                            Serial.printf("[Viewer] Tekst scroll links: kolom %d\n", textScrollCol);
                            renderTextPage();
                        }
                    }
                    // --- AFBEELDING/OVERIG: swipe naar volgend/vorig bestand ---
                    else if (currentViewerType != FILE_TYPE_TEXT && deltaY < -SWIPE_THRESHOLD && abs(deltaY) > abs(deltaX)) {
                        Serial.println("[Viewer] Swipe omhoog - volgende");
                        showNextFile();
                    }
                    else if (currentViewerType != FILE_TYPE_TEXT && deltaY > SWIPE_THRESHOLD && abs(deltaY) > abs(deltaX)) {
                        Serial.println("[Viewer] Swipe omlaag - vorige");
                        showPrevFile();
                    }
                    else if (currentViewerType == FILE_TYPE_AUDIO && abs(deltaX) > SWIPE_THRESHOLD) {
                        // Swipe links/rechts voor volume aanpassing
                        if (deltaX > 0) {
                            audioVolume = min(21, audioVolume + 3);
                        } else {
                            audioVolume = max(0, audioVolume - 3);
                        }
                        setAudioVolume(audioVolume);
                        drawVolumeBar(audioVolume);
                        Serial.printf("[Viewer] Volume swipe: %d\n", audioVolume);
                    }
                }
                viewerIsTouching = false;
            }
        }
    }
}

/*
 * Toont "Fout Bestand" foutmelding voor niet-ondersteunde bestanden
 * Wordt gebruikt in viewer wanneer een bestand niet kan worden geopend
 */
void showErrorFile() {
    // Gebruik Arduino_Canvas voor tekst rendering (CO5300 QSPI vereist dit)
    Arduino_Canvas *canvas = new Arduino_Canvas(SCREEN_WIDTH, SCREEN_HEIGHT, gfx, 0, 0);
    if (!canvas || !canvas->begin(GFX_SKIP_OUTPUT_BEGIN)) {
        Serial.println("[Viewer] Error canvas mislukt - fallback");
        if (canvas) delete canvas;
        // Fallback: probeer toch direct te tekenen
        fillScreenBlackPSRAM();
        return;
    }

    canvas->fillScreen(BLACK);
    canvas->setTextColor(RED);
    canvas->setTextSize(3);

    // Centreer de tekst "Fout Bestand"
    int16_t textWidth = 11 * 18;  // 11 karakters * ~18 pixels per karakter bij size 3
    int16_t x = (SCREEN_WIDTH - textWidth) / 2;
    int16_t y = (SCREEN_HEIGHT - 24) / 2;  // 24 pixels hoog bij size 3
    canvas->setCursor(x, y);
    canvas->println("Fout Bestand");

    // Stuur naar display
    canvas->flush();
    delete canvas;

    Serial.println("[Viewer] Fout Bestand getoond");
}

/*
 * Toont het volgende bestand in de lijst
 */
void showNextFile() {
    if (fileListCount == 0) {
        Serial.println("[Viewer] Geen bestanden in lijst");
        return;
    }

    // Ga naar volgend bestand (cyclisch)
    currentFileIndex = (currentFileIndex + 1) % fileListCount;
    selectedFilePath = fileList[currentFileIndex];

    Serial.printf("[Viewer] Volgende bestand: %s (index %d/%d)\n",
                  selectedFilePath.c_str(), currentFileIndex + 1, fileListCount);

    // Toon het bestand op basis van type
    // (fileViewerActive is al true, dus lv_timer_handler() wordt overgeslagen)
    // Pre-clear via PSRAM buffer (vereist voor CO5300 QSPI tekst rendering)
    fillScreenBlackPSRAM();

    switch (currentViewerType) {
        case FILE_TYPE_IMAGE:
            if (!showImageFromSD(selectedFilePath.c_str(), true, true)) {
                Serial.println("[Viewer] Kon afbeelding niet tonen");
                showErrorFile();
            }
            break;
        case FILE_TYPE_TEXT:
            if (!showTextFromSD(selectedFilePath.c_str())) {
                Serial.println("[Viewer] Kon tekst niet tonen");
                showErrorFile();
            }
            break;
        case FILE_TYPE_AUDIO:
            showAudioPlayer(selectedFilePath.c_str());
            break;
        default:
            showErrorFile();
            break;
    }

    // Reset touch state na laden nieuw bestand
    // BELANGRIJK: viewerInitialReleaseDetected blijft TRUE omdat de gebruiker
    // al heeft losgelaten om de swipe te kunnen maken
    viewerReleaseTime = millis();  // Reset debounce timer
    viewerIsTouching = false;
}

/*
 * Toont het vorige bestand in de lijst
 */
void showPrevFile() {
    if (fileListCount == 0) {
        Serial.println("[Viewer] Geen bestanden in lijst");
        return;
    }

    // Ga naar vorig bestand (cyclisch)
    currentFileIndex = (currentFileIndex - 1 + fileListCount) % fileListCount;
    selectedFilePath = fileList[currentFileIndex];

    Serial.printf("[Viewer] Vorig bestand: %s (index %d/%d)\n",
                  selectedFilePath.c_str(), currentFileIndex + 1, fileListCount);

    // Toon het bestand op basis van type
    // (fileViewerActive is al true, dus lv_timer_handler() wordt overgeslagen)
    // Pre-clear via PSRAM buffer (vereist voor CO5300 QSPI tekst rendering)
    fillScreenBlackPSRAM();

    switch (currentViewerType) {
        case FILE_TYPE_IMAGE:
            if (!showImageFromSD(selectedFilePath.c_str(), true, true)) {
                Serial.println("[Viewer] Kon afbeelding niet tonen");
                showErrorFile();
            }
            break;
        case FILE_TYPE_TEXT:
            if (!showTextFromSD(selectedFilePath.c_str())) {
                Serial.println("[Viewer] Kon tekst niet tonen");
                showErrorFile();
            }
            break;
        case FILE_TYPE_AUDIO:
            showAudioPlayer(selectedFilePath.c_str());
            break;
        default:
            showErrorFile();
            break;
    }

    // Reset touch state na laden nieuw bestand
    // BELANGRIJK: viewerInitialReleaseDetected blijft TRUE
    viewerReleaseTime = millis();  // Reset debounce timer
    viewerIsTouching = false;
}

/*
 * Bepaalt het bestandstype en geeft een symbool/kleur terug
 */
const char* getFileTypeSymbol(const char* filename) {
    String fn = String(filename);
    fn.toLowerCase();

    if (fn.endsWith(".jpg") || fn.endsWith(".jpeg") || fn.endsWith(".png") ||
        fn.endsWith(".gif") || fn.endsWith(".bmp") || fn.endsWith(".pcx")) {
        return LV_SYMBOL_IMAGE;  // Afbeelding icoon
    }
    else if (fn.endsWith(".mp3") || fn.endsWith(".wav") || fn.endsWith(".ogg") || fn.endsWith(".flac")) {
        return LV_SYMBOL_AUDIO;  // Audio icoon
    }
    else if (fn.endsWith(".txt") || fn.endsWith(".log") || fn.endsWith(".csv")) {
        return LV_SYMBOL_FILE;   // Tekst bestand icoon
    }
    else {
        return LV_SYMBOL_FILE;   // Generiek bestand icoon
    }
}

/*
 * Bepaalt de kleur voor het bestandstype
 */
uint32_t getFileTypeColor(const char* filename) {
    String fn = String(filename);
    fn.toLowerCase();

    if (fn.endsWith(".jpg") || fn.endsWith(".jpeg") || fn.endsWith(".png") ||
        fn.endsWith(".gif") || fn.endsWith(".bmp") || fn.endsWith(".pcx")) {
        return 0x00FF00;  // Groen voor afbeeldingen
    }
    else if (fn.endsWith(".mp3") || fn.endsWith(".wav") || fn.endsWith(".ogg") || fn.endsWith(".flac")) {
        return 0xFF8C00;  // Oranje voor audio
    }
    else if (fn.endsWith(".txt") || fn.endsWith(".log") || fn.endsWith(".csv")) {
        return 0x00BFFF;  // Lichtblauw voor tekst
    }
    else {
        return 0x808080;  // Grijs voor onbekend
    }
}

/*
 * Event handler voor bestand selectie
 * Wordt aangeroepen wanneer gebruiker op een bestand tikt
 */
void ui_event_file_clicked(lv_event_t * e) {
    lv_obj_t * btn = (lv_obj_t *)lv_event_get_target(e);

    // Haal de bestandsnaam op uit de button user data
    const char * filename = (const char *)lv_obj_get_user_data(btn);
    if (filename == NULL) {
        Serial.println("[FILES] Geen bestandsnaam gevonden");
        return;
    }

    Serial.printf("[FILES] Bestand geselecteerd: %s\n", filename);

    // Sla het pad op
    selectedFilePath = String(filename);

    // Registreer activiteit
    registerActivity();

    // Bepaal bestandstype met de enum
    FileType fileType = getFileTypeEnum(filename);
    Serial.printf("[FILES] Bestandstype: %d\n", fileType);

    // Check of bestand bestaat
    if (!SD.exists(filename)) {
        Serial.println("[FILES] WAARSCHUWING: Bestand niet gevonden op SD kaart!");
        return;
    }

    // Verwerk op basis van bestandstype
    // BELANGRIJK: We tekenen NIET hier! We stellen het uit tot NA lv_timer_handler()
    // om te voorkomen dat LVGL over onze tekeningen rendert.
    switch (fileType) {
        case FILE_TYPE_IMAGE:
            // Toon afbeelding (JPG, JPEG, PNG, GIF, BMP, PCX)
            Serial.printf("[FILES] Pending viewer voor afbeelding: %s\n", filename);

            // Stel viewer type in
            currentViewerType = FILE_TYPE_IMAGE;

            // Bouw lijst van alle afbeeldingen voor navigatie
            buildFileListByType(FILE_TYPE_IMAGE);

            // KRITIEK: Reset ALLE touch state VOORDAT we naar viewer gaan
            FT3168->IIC_Interrupt_Flag = false;
            viewerIsTouching = false;
            viewerInitialReleaseDetected = false;
            viewerReleaseTime = 0;
            viewerTouchStartTime = 0;
            viewerTouchStartX = 0;
            viewerTouchStartY = 0;
            isTouching = false;
            touchHandled = true;

            // Zet timer VOORDAT viewer actief wordt
            viewerActivatedTime = millis();

            // STEL TEKENING UIT - wordt verwerkt in loop() NA lv_timer_handler()
            pendingViewerOpen = true;
            pendingViewerType = FILE_TYPE_IMAGE;
            pendingViewerFile = String(filename);
            Serial.println("[FILES] Viewer uitgesteld tot na LVGL render");
            break;

        case FILE_TYPE_TEXT:
            // Toon tekst bestand (TXT, LOG, CSV)
            Serial.printf("[FILES] Pending viewer voor tekst: %s\n", filename);

            currentViewerType = FILE_TYPE_TEXT;
            buildFileListByType(FILE_TYPE_TEXT);

            // Reset alle touch state
            FT3168->IIC_Interrupt_Flag = false;
            viewerIsTouching = false;
            viewerInitialReleaseDetected = false;
            viewerReleaseTime = 0;
            isTouching = false;
            touchHandled = true;
            viewerActivatedTime = millis();

            // STEL TEKENING UIT
            pendingViewerOpen = true;
            pendingViewerType = FILE_TYPE_TEXT;
            pendingViewerFile = String(filename);
            Serial.println("[FILES] Viewer uitgesteld tot na LVGL render");
            break;

        case FILE_TYPE_AUDIO:
            // Speel audio bestand (MP3, WAV, OGG, FLAC)
            Serial.printf("[FILES] Pending viewer voor audio: %s\n", filename);

            currentViewerType = FILE_TYPE_AUDIO;
            buildFileListByType(FILE_TYPE_AUDIO);

            // Reset alle touch state
            FT3168->IIC_Interrupt_Flag = false;
            viewerIsTouching = false;
            viewerInitialReleaseDetected = false;
            viewerReleaseTime = 0;
            isTouching = false;
            touchHandled = true;
            viewerActivatedTime = millis();

            // STEL TEKENING UIT
            pendingViewerOpen = true;
            pendingViewerType = FILE_TYPE_AUDIO;
            pendingViewerFile = String(filename);
            Serial.println("[FILES] Viewer uitgesteld tot na LVGL render");
            break;

        default:
            // Niet-ondersteund bestandstype - toon foutmelding IN HET BESTANDEN SCHERM
            Serial.printf("[FILES] Niet-ondersteund bestandstype: %s\n", filename);

            // Toon "Fout Bestand" in het bestanden scherm (tijdelijke melding)
            showFilesScreenError("Fout Bestand");

            // Blijf in het bestanden scherm
            break;
    }
}

/*
 * Toont tijdelijke foutmelding in het bestanden scherm
 * Melding verdwijnt na 2 seconden automatisch door lv_timer
 */
void showFilesScreenError(const char* message) {
    Serial.printf("[FILES] Foutmelding: %s\n", message);

    // Tijdelijk de info label gebruiken voor foutmelding
    lv_label_set_text(ui_files_info, message);
    lv_obj_set_style_text_color(ui_files_info, lv_color_hex(0xFF0000), 0);  // Rood

    // Na 2 seconden terugzetten naar normale tekst
    // Gebruik een one-shot timer
    lv_timer_t * timer = lv_timer_create([](lv_timer_t * timer) {
        // Reset info label naar normaal
        lv_label_set_text(ui_files_info, "1 sec drukken om te openen");
        lv_obj_set_style_text_color(ui_files_info, lv_color_hex(0x808080), 0);  // Grijs
        lv_timer_delete(timer);
    }, 2000, NULL);
    lv_timer_set_repeat_count(timer, 1);
}

/*
 * Event handler voor folder navigatie
 * Wordt aangeroepen bij lang drukken op een folder
 */
void ui_event_folder_clicked(lv_event_t * e) {
    lv_obj_t * btn = (lv_obj_t *)lv_event_get_target(e);

    // Haal het pad op uit de button user data
    const char * folderPath = (const char *)lv_obj_get_user_data(btn);
    if (folderPath == NULL) {
        Serial.println("[FILES] Geen folder pad gevonden");
        return;
    }

    Serial.printf("[FILES] Folder geselecteerd: %s\n", folderPath);

    // Navigeer naar de folder
    navigateToFolder(folderPath);
}

/*
 * Navigeer naar een folder
 * Werkt currentDirectory bij en ververst de lijst
 */
void navigateToFolder(const char* folderPath) {
    Serial.printf("[FILES] Navigeren naar: %s\n", folderPath);

    // Update huidige directory
    currentDirectory = String(folderPath);

    // Zorg dat het pad eindigt op /
    if (!currentDirectory.endsWith("/")) {
        currentDirectory += "/";
    }

    Serial.printf("[FILES] Nieuwe directory: %s\n", currentDirectory.c_str());

    // Ververs de lijst
    populateFileList();
}

/*
 * Vult de bestandenlijst met bestanden van de SD kaart
 * Ondersteunt folder navigatie met currentDirectory
 */
void populateFileList() {
    // Check of SD kaart beschikbaar is
    if (!sdCardAvailable) {
        lv_label_set_text(ui_files_sd_status, "SD Kaart: Niet gevonden");
        lv_obj_set_style_text_color(ui_files_sd_status, lv_color_hex(0xFF0000), 0);
        Serial.println("[FILES] SD kaart niet beschikbaar");
        return;
    }

    // Update SD status label met huidige map
    char statusBuf[128];
    if (currentDirectory == "/") {
        snprintf(statusBuf, sizeof(statusBuf), "SD: %.1f MB  [/]", (float)sdCardSize / (1024 * 1024));
    } else {
        // Toon alleen de laatste folder naam
        String displayPath = currentDirectory;
        if (displayPath.endsWith("/")) displayPath = displayPath.substring(0, displayPath.length() - 1);
        int lastSlash = displayPath.lastIndexOf('/');
        if (lastSlash >= 0) displayPath = displayPath.substring(lastSlash + 1);
        snprintf(statusBuf, sizeof(statusBuf), "SD: [%s]", displayPath.c_str());
    }
    lv_label_set_text(ui_files_sd_status, statusBuf);
    lv_obj_set_style_text_color(ui_files_sd_status, lv_color_hex(0x00FF00), 0);

    // Verwijder alle bestaande items uit de lijst
    lv_obj_clean(ui_files_list);

    // Open huidige directory
    File dir = SD.open(currentDirectory.c_str());
    if (!dir) {
        Serial.printf("[FILES] Kon directory niet openen: %s\n", currentDirectory.c_str());
        lv_label_set_text(ui_files_sd_status, "Map: Leesfout");
        lv_obj_set_style_text_color(ui_files_sd_status, lv_color_hex(0xFF0000), 0);
        // Ga terug naar root
        currentDirectory = "/";
        return;
    }

    // Tel items voor debug
    int folderCount = 0;
    int fileCount = 0;

    // === STAP 1: Voeg ".." toe om terug te gaan (niet in root) ===
    if (currentDirectory != "/") {
        // Bereken parent directory
        String parentDir = currentDirectory;
        if (parentDir.endsWith("/")) parentDir = parentDir.substring(0, parentDir.length() - 1);
        int lastSlash = parentDir.lastIndexOf('/');
        if (lastSlash <= 0) {
            parentDir = "/";
        } else {
            parentDir = parentDir.substring(0, lastSlash + 1);
        }

        Serial.printf("[FILES] Parent directory: %s\n", parentDir.c_str());

        // Maak ".." button
        lv_obj_t * btnBack = lv_list_add_button(ui_files_list, NULL, LV_SYMBOL_DIRECTORY "  ..");

        // Sla parent pad op
        char* pathCopy = (char*)malloc(parentDir.length() + 1);
        if (pathCopy) {
            strcpy(pathCopy, parentDir.c_str());
            lv_obj_set_user_data(btnBack, pathCopy);
        }

        // Gele kleur voor terug-knop
        lv_obj_set_style_text_color(btnBack, lv_color_hex(0xFFFF00), LV_PART_MAIN);

        // Korte klik voor ".." (geen lang drukken nodig)
        lv_obj_add_event_cb(btnBack, ui_event_folder_clicked, LV_EVENT_CLICKED, NULL);

        // Stijl
        lv_obj_set_style_bg_color(btnBack, lv_color_hex(0x3A3A2A), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(btnBack, 200, LV_PART_MAIN);
        lv_obj_set_style_text_font(btnBack, &lv_font_montserrat_18, LV_PART_MAIN);
        lv_obj_set_style_pad_ver(btnBack, 12, LV_PART_MAIN);
    }

    // === STAP 2: Eerst alle folders toevoegen ===
    File file = dir.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            const char* fname = file.name();

            // Sla systeem directories over
            if (strcmp(fname, "System Volume Information") == 0 ||
                strcmp(fname, ".") == 0 || strcmp(fname, "..") == 0) {
                file = dir.openNextFile();
                continue;
            }

            // Maak volledig pad
            String fullPath = currentDirectory;
            if (!fullPath.endsWith("/")) fullPath += "/";
            fullPath += String(fname);

            // Maak button met folder icoon
            char labelText[128];
            snprintf(labelText, sizeof(labelText), LV_SYMBOL_DIRECTORY "  %s", fname);

            lv_obj_t * btn = lv_list_add_button(ui_files_list, NULL, labelText);

            // Sla pad op als user data
            char* pathCopy = (char*)malloc(fullPath.length() + 1);
            if (pathCopy) {
                strcpy(pathCopy, fullPath.c_str());
                lv_obj_set_user_data(btn, pathCopy);
            }

            // Blauwe kleur voor folders
            lv_obj_set_style_text_color(btn, lv_color_hex(0x00BFFF), LV_PART_MAIN);

            // Klik om folder te openen
            lv_obj_add_event_cb(btn, ui_event_folder_clicked, LV_EVENT_CLICKED, NULL);

            // Stijl
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x2A2A3A), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn, 200, LV_PART_MAIN);
            lv_obj_set_style_text_font(btn, &lv_font_montserrat_18, LV_PART_MAIN);
            lv_obj_set_style_pad_ver(btn, 12, LV_PART_MAIN);

            folderCount++;
            Serial.printf("[FILES] Folder: %s\n", fullPath.c_str());
        }
        file = dir.openNextFile();
    }

    // Sluit en heropen voor tweede doorloop (bestanden)
    dir.close();
    dir = SD.open(currentDirectory.c_str());

    // === STAP 3: Dan alle bestanden toevoegen ===
    file = dir.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            const char* fname = file.name();

            // Maak volledig pad
            String fullPath = currentDirectory;
            if (!fullPath.endsWith("/")) fullPath += "/";
            fullPath += String(fname);

            // Krijg bestandstype symbool en kleur
            const char* symbol = getFileTypeSymbol(fname);
            uint32_t color = getFileTypeColor(fname);

            // Voeg button toe aan de lijst met symbool
            char labelText[128];
            snprintf(labelText, sizeof(labelText), "%s  %s", symbol, fname);

            lv_obj_t * btn = lv_list_add_button(ui_files_list, NULL, labelText);

            // Sla het volledige pad op als user data
            char* pathCopy = (char*)malloc(fullPath.length() + 1);
            if (pathCopy) {
                strcpy(pathCopy, fullPath.c_str());
                lv_obj_set_user_data(btn, pathCopy);
            }

            // Stel de kleur in op basis van bestandstype
            lv_obj_set_style_text_color(btn, lv_color_hex(color), LV_PART_MAIN);

            // Klik om bestand te openen
            lv_obj_add_event_cb(btn, ui_event_file_clicked, LV_EVENT_CLICKED, NULL);

            // Stijl aanpassingen voor betere leesbaarheid
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn, 200, LV_PART_MAIN);
            lv_obj_set_style_text_font(btn, &lv_font_montserrat_18, LV_PART_MAIN);
            lv_obj_set_style_pad_ver(btn, 12, LV_PART_MAIN);

            fileCount++;
            Serial.printf("[FILES] Bestand: %s\n", fullPath.c_str());
        }

        file = dir.openNextFile();
    }

    dir.close();

    Serial.printf("[FILES] Totaal: %d folders, %d bestanden\n", folderCount, fileCount);

    // Update info label
    int totalCount = folderCount + fileCount;
    if (totalCount == 0) {
        lv_label_set_text(ui_files_info, "Map is leeg");
    } else {
        lv_label_set_text(ui_files_info, "Tik om te openen");
    }
    lv_obj_set_style_text_color(ui_files_info, lv_color_hex(0x808080), 0);  // Grijs
}

// ============================================================================
// FUNCTIE: initAudio
// ============================================================================
/*
 * Initialiseert de ES8311 audio codec en I2S interface.
 *
 * De ES8311 is een high-performance audio codec met:
 * - 24-bit ADC/DAC
 * - I2S interface voor audio data
 * - I2C interface voor configuratie
 *
 * Returns:
 *   true  - Initialisatie succesvol
 *   false - Fout: audio codec niet gevonden
 */
/*
 * ES8311 Register schrijven via I2C
 */
void es8311WriteReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ES8311_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

/*
 * ES8311 Register lezen via I2C
 */
uint8_t es8311ReadReg(uint8_t reg) {
    Wire.beginTransmission(ES8311_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(ES8311_I2C_ADDR, (uint8_t)1);
    return Wire.read();
}

/*
 * ES8311 Codec initialisatie
 * --------------------------
 * Configureert de ES8311 audio codec voor playback via I2S.
 * Gebaseerd op ES8311 datasheet en Espressif ESP-ADF driver.
 */
bool initES8311() {
    Serial.println("[Audio] ES8311 codec initialiseren...");

    // Check of ES8311 aanwezig is op I2C bus
    Wire.beginTransmission(ES8311_I2C_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("[Audio] ES8311 niet gevonden op I2C!");
        return false;
    }
    Serial.printf("[Audio] ES8311 gevonden op adres 0x%02X\n", ES8311_I2C_ADDR);

    // Lees chip ID (register 0xFD) - verwacht 0x83 voor ES8311
    uint8_t chipId = es8311ReadReg(0xFD);
    Serial.printf("[Audio] ES8311 Chip ID: 0x%02X (verwacht: 0x83)\n", chipId);

    // ================================================================
    // STAP 1: Software Reset
    // ================================================================
    es8311WriteReg(0x00, 0x1F);  // Reset alle registers
    delay(20);
    es8311WriteReg(0x00, 0x80);  // Uit reset, CSM_ON=1 (sequencer aan)
    delay(20);

    // ================================================================
    // STAP 2: Clock Configuratie
    // ================================================================
    // Reg 0x01: Clk Manager - MCLK source selectie
    // Bit 7: MCLK van extern (0) of intern (1)
    // Bit 6: Bclk inversie
    // Bit 5-0: MCLK divider
    es8311WriteReg(0x01, 0x3F);  // Alle clocks aan, MCLK van extern

    // Reg 0x02: Clk Manager - MCLK/LRCK ratio
    // Voor MCLK=256*Fs, ratio = 256
    es8311WriteReg(0x02, 0x00);  // Pre-divider = 1

    // Reg 0x03-0x05: Clk dividers
    es8311WriteReg(0x03, 0x10);  // ADC_OSR = 16x
    es8311WriteReg(0x04, 0x10);  // DAC_OSR = 16x
    es8311WriteReg(0x05, 0x00);  // Clk divider

    // Reg 0x06: Clk Manager - Clock gating
    es8311WriteReg(0x06, 0x0F);  // Alle clocks enabled

    // Reg 0x07-0x08: Extra clk config
    es8311WriteReg(0x07, 0x00);
    es8311WriteReg(0x08, 0xFF);

    // ================================================================
    // STAP 3: I2S/SDP Configuratie
    // ================================================================
    // Reg 0x09: SDP IN (data naar ES8311)
    // Bits 7-6: I2S format (00=I2S, 01=Left, 10=Right, 11=DSP)
    // Bits 5-4: Word length (00=24bit, 01=20bit, 10=18bit, 11=16bit)
    // Bits 3-2: BCLK polarity
    // Bits 1-0: Channel polarity
    es8311WriteReg(0x09, 0x0C);  // I2S mode, 16-bit

    // Reg 0x0A: SDP OUT (data van ES8311)
    es8311WriteReg(0x0A, 0x0C);  // I2S mode, 16-bit

    // ================================================================
    // STAP 4: System Power Up
    // ================================================================
    // Reg 0x0B: System
    es8311WriteReg(0x0B, 0x00);

    // Reg 0x0C: System
    es8311WriteReg(0x0C, 0x00);

    // Reg 0x0D: Power management - POWER UP DAC EN ADC
    // Bit 0: Enable reference
    // Bit 1: Enable analog
    // Bit 2: Enable IBIAS
    es8311WriteReg(0x0D, 0x07);  // Power up: ref + analog + ibias

    // Reg 0x0E: Power management
    // Bit 0-1: DAC/ADC power up
    es8311WriteReg(0x0E, 0x03);  // Power up DAC en ADC

    // Reg 0x0F-0x11: Analog config
    es8311WriteReg(0x0F, 0x00);  // LP mode uit
    es8311WriteReg(0x10, 0x00);
    es8311WriteReg(0x11, 0x00);

    // ================================================================
    // STAP 5: ADC Configuratie (microfoon input)
    // ================================================================
    es8311WriteReg(0x12, 0x28);  // ADC config - MIC input
    es8311WriteReg(0x13, 0x00);  // ADC volume
    es8311WriteReg(0x14, 0x1A);  // ADC power up

    // ================================================================
    // STAP 6: DAC Configuratie (speaker output) - KRITIEK!
    // ================================================================
    // Reg 0x15: DAC config
    // Bit 7: DAC_MUTE (0=niet gedempt, 1=gedempt)
    // Bit 6: DAC soft mute
    es8311WriteReg(0x15, 0x00);  // DAC niet gedempt

    // Reg 0x16: DAC mono/stereo
    es8311WriteReg(0x16, 0x00);  // Stereo mode

    // Reg 0x17: DAC volume (0x00 = max, 0xFF = min)
    es8311WriteReg(0x17, 0x00);  // MAX VOLUME!

    // Reg 0x18-0x1C: DAC extra config
    es8311WriteReg(0x18, 0x00);
    es8311WriteReg(0x19, 0x00);
    es8311WriteReg(0x1A, 0x00);
    es8311WriteReg(0x1B, 0x00);
    es8311WriteReg(0x1C, 0x00);

    // ================================================================
    // STAP 7: Output Configuratie - SPEAKER ENABLE
    // ================================================================
    // Reg 0x32: DAC gain
    es8311WriteReg(0x32, 0xC0);  // +24dB gain voor speaker

    // Reg 0x37: Output mixer
    es8311WriteReg(0x37, 0x08);  // DAC naar output

    // Reg 0x44: GPIO config
    es8311WriteReg(0x44, 0x08);  // GPIO2 als output

    // Reg 0x45: GP Control register
    // Dit register controleert de speaker output!
    es8311WriteReg(0x45, 0x00);  // Enable speaker output

    // ================================================================
    // Verificatie: Lees belangrijke registers terug
    // ================================================================
    Serial.println("[Audio] ES8311 register verificatie:");
    Serial.printf("  Reg 0x00 (Reset): 0x%02X (verwacht: 0x80)\n", es8311ReadReg(0x00));
    Serial.printf("  Reg 0x0D (Power): 0x%02X (verwacht: 0x07)\n", es8311ReadReg(0x0D));
    Serial.printf("  Reg 0x0E (Power): 0x%02X (verwacht: 0x03)\n", es8311ReadReg(0x0E));
    Serial.printf("  Reg 0x15 (DAC):   0x%02X (verwacht: 0x00)\n", es8311ReadReg(0x15));
    Serial.printf("  Reg 0x17 (Vol):   0x%02X (verwacht: 0x00)\n", es8311ReadReg(0x17));
    Serial.printf("  Reg 0x32 (Gain):  0x%02X (verwacht: 0xC0)\n", es8311ReadReg(0x32));

    Serial.println("[Audio] ES8311 geïnitialiseerd voor speaker output");
    return true;
}

bool initAudio() {
    Serial.println("[Audio] Audio subsysteem initialiseren...");

    // ====================================================================
    // STAP 1: Configureer Power Amplifier (UIT bij start)
    // ====================================================================
    pinMode(PA_CTRL, OUTPUT);
    digitalWrite(PA_CTRL, LOW);  // Amplifier UIT - voorkomt ruis bij boot
    Serial.println("[Audio] Power amplifier geconfigureerd (UIT)");

    // ====================================================================
    // STAP 2: Start MCLK met LEDC - ES8311 heeft MCLK nodig!
    // ====================================================================
    // MCLK = 256 x Fs = 256 x 44100 = 11.2896 MHz
    const uint32_t mclkFreq = 11289600;
    if (ledcAttach(I2S_MCLK, mclkFreq, 1)) {
        ledcWrite(I2S_MCLK, 1);  // 50% duty cycle voor MCLK
        Serial.printf("[Audio] MCLK gestart op pin %d: %.2f MHz\n", I2S_MCLK, mclkFreq / 1000000.0);
    } else {
        Serial.println("[Audio] FOUT: MCLK kon niet gestart worden!");
        return false;
    }
    delay(100);  // Wacht op MCLK stabilisatie

    // ====================================================================
    // STAP 3: Initialiseer ES8311 codec via I2C
    // ====================================================================
    if (!initES8311()) {
        Serial.println("[Audio] WAARSCHUWING: ES8311 init mislukt");
    }

    // ====================================================================
    // STAP 4: Initialiseer I2S bij boot (voorkomt ruis bij eerste play)
    // ====================================================================
    // Door setPinout() bij boot aan te roepen wordt I2S correct geïnitialiseerd.
    // De "i2s_channel_disable" warning is onschadelijk - het betekent alleen
    // dat er nog geen channel was om te disablen.
    Serial.println("[Audio] I2S pins configureren (bij boot)...");
    audio.setPinout(I2S_BCLK, I2S_LRCK, I2S_DOUT, I2S_DIN);
    delay(50);  // Wacht op I2S stabilisatie
    Serial.println("[Audio] I2S geconfigureerd");

    // Stel standaard volume in
    audioVolume = 21;
    audio.setVolume(audioVolume);

    audioAvailable = true;

    Serial.println("[Audio] ========= AUDIO CONFIGURATIE =========");
    Serial.printf("[Audio] ES8311 codec op I2C adres: 0x%02X\n", ES8311_I2C_ADDR);
    Serial.printf("[Audio] I2S MCLK: pin %d (LEDC @ %.2f MHz)\n", I2S_MCLK, mclkFreq / 1000000.0);
    Serial.printf("[Audio] I2S BCLK: pin %d\n", I2S_BCLK);
    Serial.printf("[Audio] I2S LRCK: pin %d\n", I2S_LRCK);
    Serial.printf("[Audio] I2S DOUT: pin %d (data naar ES8311)\n", I2S_DOUT);
    Serial.printf("[Audio] I2S DIN:  pin %d (data van ES8311)\n", I2S_DIN);
    Serial.printf("[Audio] PA_CTRL:  pin %d (speaker enable)\n", PA_CTRL);
    Serial.printf("[Audio] Volume:   %d/21\n", audioVolume);
    Serial.println("[Audio] ========================================");

    return true;
}

/*
 * Stel audio volume in (0-21)
 */
void setAudioVolume(uint8_t volume) {
    if (volume > 21) volume = 21;
    audioVolume = volume;

    audio.setVolume(audioVolume);

    Serial.printf("[Audio] Volume: %d/21\n", audioVolume);
}

/*
 * Start audio playback van SD kaart
 * I2S wordt bij boot geconfigureerd in initAudio()
 */
void startAudioPlayback(const char* filename) {
    if (!audioAvailable) {
        Serial.println("[Audio] Audio niet beschikbaar");
        return;
    }

    // Stop eventuele huidige playback
    if (audioPlaying) {
        audio.stopSong();
        audioPlaying = false;
    }
    audioPaused = false;  // Reset pause status bij nieuwe playback

    Serial.printf("[Audio] ====== START PLAYBACK ======\n");
    Serial.printf("[Audio] Bestand: %s\n", filename);
    Serial.printf("[Audio] Heap vrij: %d, PSRAM vrij: %d\n",
                  ESP.getFreeHeap(), ESP.getFreePsram());

    // Check of bestand bestaat op SD kaart
    File testFile = SD.open(filename, FILE_READ);
    if (!testFile) {
        Serial.printf("[Audio] FOUT: Bestand niet gevonden op SD: %s\n", filename);
        audioPlaying = false;
        return;
    }
    Serial.printf("[Audio] Bestand gevonden, grootte: %d bytes\n", testFile.size());
    testFile.close();

    // ====================================================================
    // STAP 1: Activeer Power Amplifier
    // ====================================================================
    // I2S is al geconfigureerd bij boot in initAudio()
    digitalWrite(PA_CTRL, HIGH);
    Serial.println("[Audio] Power amplifier AAN");
    delay(100);  // Wacht op PA stabilisatie (verlengd van 50 naar 100ms)

    // ====================================================================
    // STAP 2: Stel volume in (opnieuw, voor zekerheid)
    // ====================================================================
    audio.setVolume(audioVolume);
    Serial.printf("[Audio] Volume ingesteld: %d/21\n", audioVolume);

    // ====================================================================
    // STAP 3: Start playback van SD kaart
    // ====================================================================
    Serial.printf("[Audio] Verbinden met bestand via connecttoFS...\n");
    if (audio.connecttoFS(SD, filename)) {
        audioPlaying = true;
        currentAudioFile = String(filename);
        Serial.println("[Audio] Playback GESTART!");
        Serial.printf("[Audio] Volume: %d/21, audioPlaying: %d\n", audioVolume, audioPlaying);
    } else {
        Serial.println("[Audio] FOUT: connecttoFS() gaf FALSE terug!");
        Serial.printf("[Audio] Bestand pad: '%s'\n", filename);
        audioPlaying = false;
        digitalWrite(PA_CTRL, LOW);
    }
    Serial.printf("[Audio] Heap na start: %d, PSRAM: %d\n",
                  ESP.getFreeHeap(), ESP.getFreePsram());
    Serial.printf("[Audio] =============================\n");
}

/*
 * Stop audio playback
 */
void stopAudioPlayback() {
    if (audioPlaying) {
        audio.stopSong();
        audioPlaying = false;
        audioPaused = false;  // Reset pause status bij stoppen
        currentAudioFile = "";
        Serial.println("[Audio] Playback gestopt");
    }

    // Schakel Power Amplifier uit (voorkomt ruis)
    // MCLK blijft draaien (laag stroomverbruik, nodig voor ES8311)
    digitalWrite(PA_CTRL, LOW);
    Serial.println("[Audio] Power amplifier UIT");
}

/*
 * Audio loop - moet in main loop worden aangeroepen
 */
void audioLoop() {
    if (audioAvailable && audioPlaying) {
        audio.loop();
    }
}

/*
 * Toggle audio playback (pause/resume)
 * Gebruikt audio.pauseResume() van de ESP32-audioI2S library.
 * Update de audioPaused status en hertekent de play/pause knop.
 */
void toggleAudioPlayback() {
    if (!audioAvailable || !audioPlaying) {
        Serial.println("[Audio] Toggle: audio niet actief, negeer");
        return;
    }

    // Toggle pause/resume via library
    audio.pauseResume();
    audioPaused = !audioPaused;

    // Herteken de play/pause knop (paused = toon play driehoek, playing = toon pause balken)
    drawControlButtons(!audioPaused);

    Serial.printf("[Audio] %s\n", audioPaused ? "GEPAUZEERD" : "HERVAT");
}

/*
 * Verwerkt een tap op het audio player scherm.
 * Controleert of de tap binnen een van de knop-gebieden valt.
 *
 * Button layout (komt overeen met drawControlButtons):
 *   BTN_Y = 360, BTN_SIZE = 75, BTN_STEP = 90, startX = 32
 *   Prev:       x=32..107,   y=360..435
 *   Play/Pause: x=122..197,  y=360..435
 *   Next:       x=212..287,  y=360..435
 *   Repeat:     x=302..377,  y=360..435
 *
 * Volume bar: Y=450, breedte=260, gecentreerd
 *   "-" links:  x=35..75,    y=440..475
 *   "+" rechts: x=335..385,  y=440..475
 *
 * Parameters:
 *   x, y: touch coördinaten van de tap
 */
void handleAudioTap(int16_t x, int16_t y) {
    // Button layout constanten (moeten overeenkomen met drawControlButtons)
    const int btnY = 360;
    const int btnSize = 75;
    const int btnStep = 90;
    const int startX = 32;
    const int hitMargin = 10;  // Extra marge rondom knoppen

    // Check of tap in de Y-range van de knoppen valt (360-10 tot 435+10)
    if (y >= btnY - hitMargin && y <= btnY + btnSize + hitMargin) {

        // ===== VORIGE KNOP (x=32..107) =====
        if (x >= startX - hitMargin && x < startX + btnSize + hitMargin) {
            Serial.printf("[Audio Tap] VORIGE op (%d,%d)\n", x, y);
            showPrevFile();
            return;
        }

        // ===== PLAY/PAUSE KNOP (x=122..197) =====
        int playX = startX + btnStep;
        if (x >= playX - hitMargin && x < playX + btnSize + hitMargin) {
            Serial.printf("[Audio Tap] PLAY/PAUSE op (%d,%d)\n", x, y);
            toggleAudioPlayback();
            return;
        }

        // ===== VOLGENDE KNOP (x=212..287) =====
        int nextBtnX = startX + 2 * btnStep;
        if (x >= nextBtnX - hitMargin && x < nextBtnX + btnSize + hitMargin) {
            Serial.printf("[Audio Tap] VOLGENDE op (%d,%d)\n", x, y);
            showNextFile();
            return;
        }

        // ===== HERHAAL KNOP (x=302..377) =====
        int repeatX = startX + 3 * btnStep;
        if (x >= repeatX - hitMargin && x < repeatX + btnSize + hitMargin) {
            Serial.printf("[Audio Tap] HERHAAL op (%d,%d)\n", x, y);
            // Herstart het huidige nummer
            if (audioAvailable && !selectedFilePath.isEmpty()) {
                startAudioPlayback(selectedFilePath.c_str());
                audioPaused = false;
                drawControlButtons(true);  // Toon pause knop
            }
            return;
        }
    }

    // ===== VOLUME CONTROLS =====
    int barX = (SCREEN_WIDTH - 260) / 2;  // 75
    int barY = 450;

    // Volume "-" (links van balk, x=35..75, y=440..475)
    if (x >= barX - 40 && x < barX && y >= barY - 10 && y <= barY + 30) {
        audioVolume = max(0, audioVolume - 1);
        setAudioVolume(audioVolume);
        drawVolumeBar(audioVolume);
        Serial.printf("[Audio Tap] Volume omlaag: %d\n", audioVolume);
        return;
    }

    // Volume "+" (rechts van balk, x=335..385, y=440..475)
    if (x > barX + 260 && x < barX + 310 && y >= barY - 10 && y <= barY + 30) {
        audioVolume = min(21, audioVolume + 1);
        setAudioVolume(audioVolume);
        drawVolumeBar(audioVolume);
        Serial.printf("[Audio Tap] Volume omhoog: %d\n", audioVolume);
        return;
    }

    Serial.printf("[Audio Tap] Geen knop geraakt op (%d,%d)\n", x, y);
}

// ============================================================================
// FUNCTIE: initTouch
// ============================================================================
/*
 * Initialiseert de FT3168 touch controller.
 * Configureert de controller in monitor mode voor laag stroomverbruik.
 *
 * Returns:
 *   true  - Initialisatie succesvol
 *   false - Fout: touch controller niet gevonden
 *
 * Power Modes van FT3168:
 * - TOUCH_POWER_ACTIVE: Continu scannen, hoogste verbruik (~15mA)
 * - TOUCH_POWER_MONITOR: Laag verbruik, wakker bij touch (~3mA)
 * - TOUCH_POWER_HIBERNATE: Zeer laag verbruik, handmatige wake (~1μA)
 *
 * We gebruiken MONITOR mode voor goede balans tussen
 * responsiviteit en stroomverbruik.
 *
 * Bij falen:
 * - Error wordt naar Serial geprint
 * - Programma kan doorgaan maar touch werkt niet
 * - false return waarde kan gebruikt worden voor error handling
 */
bool initTouch() {
    Serial.println("[Touch] FT3168 initialiseren...");

    // Probeer touch controller te starten
    // begin() zoekt naar device op I2C bus
    if (!FT3168->begin()) {
        // Touch controller niet gevonden op I2C bus
        Serial.println("[Touch] ERROR: FT3168 niet gevonden!");
        return false;
    }

    // Configureer power mode naar monitor
    // Dit zorgt voor laag verbruik met interrupt-based wake
    FT3168->IIC_Write_Device_State(
        FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
        FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_MONITOR
    );

    Serial.println("[Touch] FT3168 OK");
    return true;
}

// ============================================================================
// SETUP FUNCTIE - Eenmalige initialisatie bij opstart
// ============================================================================
/*
 * Arduino setup() functie - wordt eenmaal uitgevoerd bij opstart.
 *
 * Initialisatie Volgorde (BELANGRIJK!):
 * ------------------------------------
 * De volgorde van initialisatie is kritiek voor correct functioneren:
 *
 * 1. Serial    - Debug output vanaf het begin
 * 2. I2C       - Nodig voor touch en PMU
 * 3. PMU       - Power management moet vroeg actief zijn
 * 4. Display   - Moet geïnitialiseerd zijn voor LVGL
 * 5. Touch     - Kan nu I2C gebruiken
 * 6. WiFi      - Netwerk verbinding
 * 7. NTP       - Vereist WiFi
 * 8. LVGL      - Vereist display
 * 9. UI        - Vereist LVGL
 *
 * Error Handling:
 * - Elke stap print status naar Serial
 * - Niet-kritieke fouten stoppen het programma niet
 * - Kritieke fouten (buffer allocatie) stoppen wel
 */
void setup() {
    // ========================================
    // 1. SERIELE COMMUNICATIE
    // ========================================
    // Start seriële communicatie voor debug output
    // 115200 baud is standaard voor ESP32 debugging
    Serial.begin(115200);

    // Korte wachttijd voor seriele verbinding stabilisatie
    // Vooral nodig bij USB-Serial adapters
    delay(500);

    // Print welkomstbericht met versie informatie
    Serial.println();
    Serial.println("==================================================");
    Serial.println("ESP32-S3 Smartwatch LVGL - Versie 2.3.1");
    Serial.println("Auteur: JWP van Renen");
    Serial.println("Datum:  31 januari 2026, 00:15 uur (Europa/Brussel)");
    Serial.println("==================================================");

    // ========================================
    // 2. I2C BUS INITIALISATIE
    // ========================================
    // I2C bus moet gestart worden voordat we met PMU of touch praten
    Serial.println("[I2C] Bus initialiseren...");

    // Start I2C met geconfigureerde pinnen
    // Dit initialiseert de hardware I2C peripheral
    Wire.begin(IIC_SDA, IIC_SCL);

    Serial.println("[I2C] OK");

    // ========================================
    // 3. AXP2101 POWER MANAGEMENT
    // ========================================
    // PMU initialisatie - beheert alle power voor het horloge
    Serial.println("[PMU] AXP2101 initialiseren...");

    // Probeer AXP2101 te initialiseren op I2C bus
    if (power.begin(Wire, AXP2101_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
        Serial.println("[PMU] AXP2101 OK");

        // Configureer PMU instellingen
        // Disable alle interrupts - we pollen batterij status
        power.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);

        // Clear eventuele pending interrupt flags
        power.clearIrqStatus();

        // Enable batterij detectie - nodig voor percentage meting
        power.enableBattDetection();

        // Enable batterij voltage meting - nodig voor percentage berekening
        power.enableBattVoltageMeasure();
    } else {
        // PMU niet gevonden - programma gaat door maar zonder batterij info
        Serial.println("[PMU] ERROR: AXP2101 niet gevonden!");
    }

    // ========================================
    // 4. DISPLAY INITIALISATIE
    // ========================================
    // CO5300 AMOLED display initialiseren via QSPI
    Serial.println("[Display] CO5300 initialiseren...");

    // Start display driver
    if (!gfx->begin()) {
        // Display initialisatie mislukt - ernstige fout
        Serial.println("[Display] ERROR: Display initialisatie mislukt!");
        // We gaan door, maar display zal niet werken
    } else {
        Serial.println("[Display] CO5300 OK");
    }

    // Maak scherm zwart (clear) voordat we beginnen
    // BLACK is gedefinieerd in Arduino_GFX als 0x0000
    gfx->fillScreen(BLACK);

    // Zet helderheid naar normaal niveau
    setDisplayBrightness(NORMAL_BRIGHTNESS);

    // ========================================
    // 5. TOUCH INITIALISATIE
    // ========================================
    // FT3168 capacitieve touch controller initialiseren
    initTouch();

    // ========================================
    // 6. QMI8658 IMU (ACCELEROMETER) INITIALISATIE
    // ========================================
    // Initialiseer de 6-axis IMU voor stappentelling
    // De QMI8658 bevat een accelerometer en gyroscoop
    // Wij gebruiken alleen de accelerometer voor stap detectie
    Serial.println("[IMU] Accelerometer initialiseren...");
    imuAvailable = initIMU();
    if (imuAvailable) {
        Serial.println("[IMU] QMI8658 OK - Stappenteller actief");
    } else {
        Serial.println("[IMU] WAARSCHUWING: Stappenteller niet beschikbaar");
    }

    // ========================================
    // 7. SD KAART INITIALISATIE
    // ========================================
    // MicroSD kaart initialiseren via SPI
    initSDCard();

    // Als SD kaart beschikbaar, toon bestanden in Serial
    if (sdCardAvailable) {
        listSDFiles("/", 1);  // Lijst root directory met 1 niveau diep
    }

    // ========================================
    // 7b. AUDIO INITIALISATIE
    // ========================================
    // ES8311 audio codec en I2S initialiseren
    initAudio();

    // ========================================
    // 8. WIFI VERBINDING
    // ========================================
    // Maak verbinding met geconfigureerd WiFi netwerk
    connectToWiFi();

    // ========================================
    // 9. NTP TIJD SYNCHRONISATIE
    // ========================================
    // Synchroniseer tijd alleen als WiFi verbonden is
    if (WiFi.status() == WL_CONNECTED) {
        initTime();
    } else {
        Serial.println("[NTP] Overgeslagen - geen WiFi verbinding");
    }

    // ========================================
    // 10. LVGL INITIALISATIE
    // ========================================
    // Light and Versatile Graphics Library initialiseren
    Serial.println("[LVGL] Initialiseren...");

    // Initialiseer LVGL library
    // Dit moet VOOR het aanmaken van displays en input devices
    lv_init();

    // ========================================
    // BUFFER ALLOCATIE - FULL SCREEN in PSRAM
    // ========================================
    // Voor FULL render mode hebben we een buffer nodig die het hele
    // scherm kan bevatten. Dit is ~400KB, te groot voor internal RAM.
    //
    // PSRAM wordt gebruikt voor de grote buffer.
    // Single buffering is voldoende voor FULL mode.
    Serial.println("[LVGL] Full-screen buffer alloceren uit PSRAM...");
    Serial.printf("[LVGL] Buffer grootte: %d bytes (410x502x2)\n", DRAW_BUF_SIZE);

    // Buffer alloceren uit PSRAM (8MB beschikbaar)
    draw_buf1 = (uint8_t *)heap_caps_malloc(DRAW_BUF_SIZE,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    // Geen tweede buffer nodig voor FULL mode (single buffering)
    draw_buf2 = NULL;

    // Check of buffer allocatie gelukt is
    if (!draw_buf1) {
        Serial.println("[LVGL] FATALE FOUT: Buffer allocatie mislukt!");
        return;  // Stop setup, loop() zal lege iteraties draaien
    }

    Serial.println("[LVGL] Single buffering: buffer succesvol gealloceerd");

    // Initialiseer buffer met nullen
    memset(draw_buf1, 0, DRAW_BUF_SIZE);

    // Maak LVGL display driver
    // Dit koppelt LVGL aan ons fysieke display
    lvgl_disp = lv_display_create(SCREEN_WIDTH, SCREEN_HEIGHT);

    // BELANGRIJK: Stel kleurformaat expliciet in op RGB565
    // Dit moet matchen met onze fysieke display en buffer formaat
    lv_display_set_color_format(lvgl_disp, LV_COLOR_FORMAT_RGB565);

    // Configureer display flush callback
    // LVGL roept deze functie aan wanneer pixels naar display moeten
    lv_display_set_flush_cb(lvgl_disp, my_disp_flush);

    // Configureer display buffer - FULL RENDER MODE
    // FULL mode tekent altijd het hele scherm opnieuw.
    // Dit voorkomt de "italic" tekst vervormingen die optreden
    // bij partial updates (mogelijk door stride/offset issues).
    //
    // Single buffering: draw_buf2 = NULL
    // Buffer grootte: volledig scherm (410x502x2 = ~400KB)
    lv_display_set_buffers(lvgl_disp, draw_buf1, NULL, DRAW_BUF_SIZE,
                          LV_DISPLAY_RENDER_MODE_FULL);

    // Maak LVGL input device voor touch
    lvgl_indev = lv_indev_create();

    // Configureer als pointer type (touchscreen)
    // Andere types: keyboard, encoder, button
    lv_indev_set_type(lvgl_indev, LV_INDEV_TYPE_POINTER);

    // Configureer touch read callback
    // LVGL roept deze functie aan om touch status te lezen
    lv_indev_set_read_cb(lvgl_indev, my_touchpad_read);

    // Stel long press tijd in op 1000ms (1 seconde)
    // Dit zorgt dat bestanden pas openen na 1 seconde indrukken
    lv_indev_set_long_press_time(lvgl_indev, 1000);

    Serial.println("[LVGL] OK (long press: 1 sec)");

    // ========================================
    // 11. UI INITIALISATIE
    // ========================================
    // Initialiseer de LVGL schermen (gegenereerd of handmatig)
    Serial.println("[UI] Schermen initialiseren...");

    // ui_init() maakt alle schermen en widgets aan
    // Gedefinieerd in ui.h/ui.c
    ui_init();

    // ========================================
    // BELANGRIJK: Forceer volledige display refresh
    // ========================================
    // Dit lost het probleem op van verminkde tekst bij opstarten.
    // Zonder deze refresh worden sommige UI elementen niet correct
    // gerenderd totdat er een schermwissel plaatsvindt.
    //
    // lv_obj_invalidate() markeert het object (en alle kinderen) als "dirty"
    // lv_refr_now() forceert een onmiddellijke refresh van alle dirty objecten
    lv_obj_invalidate(lv_scr_act());  // Markeer actief scherm als dirty
    lv_refr_now(NULL);                // Forceer onmiddellijke refresh

    Serial.println("[UI] OK - Display volledig gerenderd");

    // Voer initiële UI updates uit
    // WiFi status
    updateWiFiStatus(WiFi.status() == WL_CONNECTED);

    // Batterij niveau
    if (power.isBatteryConnect()) {
        // Batterij aanwezig - toon actueel percentage
        updateBatteryLevel(power.getBatteryPercent());
    } else {
        // Geen batterij gedetecteerd - toon 100% als fallback
        updateBatteryLevel(100);
    }

    // Start activity timer voor power saving
    // Vanaf nu begint de inactiviteit timeout te lopen
    lastActivityTime = millis();

    // Print voltooiingsbericht
    Serial.println();
    Serial.println("==================================================");
    Serial.println("Initialisatie compleet!");
    Serial.println("Swipe links/rechts om van scherm te wisselen");
    Serial.println("==================================================");
    Serial.println();
}

// ============================================================================
// LOOP FUNCTIE - Hoofdlus (wordt continu herhaald)
// ============================================================================
/*
 * Arduino loop() functie - wordt continu uitgevoerd na setup().
 *
 * Taken per iteratie:
 * ------------------
 * 1. LVGL tick update (timing voor animaties en timeouts)
 * 2. LVGL task handler (verwerk events, render updates)
 * 3. Power saving status check en update
 * 4. Periodieke updates (elke seconde):
 *    - Tijd display
 *    - NTP sync check
 *    - WiFi status
 *    - Batterij status (elke 30 seconden)
 *
 * Timing:
 * - Loop draait zo snel mogelijk (~200 iteraties/seconde)
 * - 5ms delay aan het eind voorkomt CPU hogging
 * - LVGL tick elke 5ms voor accurate timing
 *
 * Architectuur:
 * - Non-blocking design: geen lange delays of while loops
 * - State machine: variabelen houden state bij tussen iteraties
 * - Event-driven: touch events via interrupt flag
 */
void loop() {
    // Statische variabele voor LVGL tick timing
    // Behoudt waarde tussen loop iteraties
    static unsigned long lastTick = 0;

    // Haal huidige tijd op (milliseconden sinds boot)
    unsigned long now = millis();

    // ========================================
    // 1. LVGL TICK UPDATE
    // ========================================
    // LVGL heeft een regelmatige tick nodig voor:
    // - Animatie timing
    // - Input timeout detectie
    // - Timer callbacks
    //
    // We updaten elke 5ms of meer (afhankelijk van loop snelheid)
    if (now - lastTick >= 5) {
        // Informeer LVGL hoeveel tijd is verstreken
        lv_tick_inc(now - lastTick);

        // Update referentie voor volgende berekening
        lastTick = now;
    }

    // ========================================
    // 2. LVGL TASK HANDLER OF VIEWER TOUCH
    // ========================================
    // Verwerkt alle LVGL taken OF viewer touch input:
    // - Als viewer NIET actief: LVGL verwerkt alles (touch, animaties, rendering)
    // - Als viewer WEL actief: We verwerken touch zelf (LVGL zou tekeningen overschrijven)
    //
    if (fileViewerActive) {
        // Viewer actief: verwerk touch zelf, skip LVGL
        handleViewerTouch();
    } else {
        // Normaal: LVGL verwerkt alles
        lv_timer_handler();
    }

    // ========================================
    // 2a. PENDING VIEWER VERWERKING
    // ========================================
    // Dit wordt NA lv_timer_handler() uitgevoerd om te voorkomen
    // dat LVGL over onze gfx tekeningen rendert.
    if (pendingViewerOpen) {
        Serial.println("[VIEWER] Pending viewer verwerken...");
        pendingViewerOpen = false;  // Reset flag

        // Laad blank LVGL scherm
        lv_scr_load(ui_ScreenViewer);
        Serial.println("[VIEWER] lv_scr_load(ui_ScreenViewer) uitgevoerd");

        // KRITIEK: Forceer LVGL om het blanke viewer scherm DIRECT te renderen
        // Zonder deze call heeft LVGL een pending screen change die onze GFX
        // tekeningen later kan overschrijven. Dit is dezelfde aanpak als in
        // closeFileViewer() waar ook lv_timer_handler() na lv_scr_load() wordt aangeroepen.
        lv_timer_handler();
        Serial.println("[VIEWER] lv_timer_handler() geforceerd - blank scherm gerenderd");

        // Wacht tot display DMA transfer volledig is afgerond
        delay(50);

        // NU pas viewer mode activeren (LVGL state is volledig geflusht)
        fileViewerActive = true;
        Serial.println("[VIEWER] fileViewerActive = true (na LVGL flush)");

        // Registreer activiteit om display dimming te voorkomen tijdens laden
        registerActivity();

        // OPMERKING: Geen gfx->fillScreen(BLACK) hier.
        // Elke viewer functie handelt zijn eigen achtergrond af via
        // Arduino_Canvas of draw16bitRGBBitmap (CO5300 QSPI vereist dit).
        Serial.println("[VIEWER] Scherm klaar voor viewer rendering");

        // Teken de inhoud gebaseerd op bestandstype
        bool success = false;
        switch (pendingViewerType) {
            case FILE_TYPE_IMAGE:
                Serial.printf("[VIEWER] Afbeelding laden: %s\n", pendingViewerFile.c_str());
                success = showImageFromSD(pendingViewerFile.c_str(), true, true);
                if (success) {
                    Serial.println("[VIEWER] Afbeelding succesvol weergegeven!");
                }
                break;

            case FILE_TYPE_TEXT:
                Serial.printf("[VIEWER] Tekst laden: %s\n", pendingViewerFile.c_str());
                success = showTextFromSD(pendingViewerFile.c_str());
                if (success) {
                    Serial.println("[VIEWER] Tekst succesvol weergegeven!");
                }
                break;

            case FILE_TYPE_AUDIO:
                Serial.printf("[VIEWER] Audio spelen: %s\n", pendingViewerFile.c_str());
                showAudioPlayer(pendingViewerFile.c_str());
                success = true;  // Audio player altijd "succesvol"
                break;

            default:
                Serial.println("[VIEWER] Onbekend bestandstype!");
                break;
        }

        // Bij fout, ga terug naar bestanden scherm
        if (!success) {
            Serial.println("[VIEWER] FOUT: Kon bestand niet tonen!");
            fileViewerActive = false;
            currentViewerType = FILE_TYPE_NONE;
            lv_scr_load(ui_ScreenFiles);
            showFilesScreenError("Fout Bestand");
        }

        // Reset pending variabelen
        pendingViewerFile = "";
        pendingViewerType = FILE_TYPE_NONE;
    }

    // ========================================
    // 2b. AUDIO LOOP
    // ========================================
    // Verwerk audio streaming (nodig voor continue playback)
    audioLoop();

    // ========================================
    // 3. POWER SAVING UPDATE
    // ========================================
    // Check inactiviteit en pas display helderheid aan
    // Transities: Actief -> Gedimd -> Slaapstand
    updatePowerSaving();

    // ========================================
    // 4-6. PERIODIEKE UPDATES (elke seconde)
    // ========================================
    // Statische variabele voor timing van seconde-updates
    static unsigned long lastTimeUpdate = 0;

    // Update elke seconde EN alleen als display niet in slaapstand is
    // (geen updates nodig als niemand kijkt)
    if (!displayAsleep && (now - lastTimeUpdate >= 1000)) {
        // Update referentie tijd
        lastTimeUpdate = now;

        // Haal huidige lokale tijd op
        struct tm timeinfo;
        if (getLocalTime(&timeinfo, 10)) {  // 10ms timeout
            // Update tijd display (uren, minuten, seconden)
            updateTimeDisplay(timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

            // Update datum display
            // tm_year is jaren sinds 1900, dus +1900 voor volledig jaar
            updateDateDisplay(timeinfo.tm_mday, timeinfo.tm_mon,
                            timeinfo.tm_year + 1900, timeinfo.tm_wday);
        }

        // Check of NTP synchronisatie nodig is
        checkNtpSync();

        // Update WiFi status op instellingenscherm
        updateWiFiStatus(WiFi.status() == WL_CONNECTED);

        // Batterij update elke 30 seconden
        // Minder frequent omdat batterij percentage langzaam verandert
        static unsigned long lastBatt = 0;
        if (now - lastBatt >= 30000) {  // 30 seconden
            lastBatt = now;

            // Update alleen als batterij aanwezig
            if (power.isBatteryConnect()) {
                updateBatteryLevel(power.getBatteryPercent());
            }
        }
    }

    // ========================================
    // ACCELEROMETER STAPPEN DETECTIE
    // ========================================
    // Lees accelerometer data van QMI8658 en detecteer stappen
    // De stap detectie algoritme werkt als volgt:
    // 1. Lees X, Y, Z acceleratie waarden
    // 2. Bereken de vector magnitude (sqrt(x²+y²+z²))
    // 3. Pas een low-pass filter toe voor smoothing
    // 4. Detecteer pieken en dalen in de gefilterde data
    // 5. Een stap = peak gevolgd door valley met voldoende amplitude
    //
    // Timing: Lees accelerometer elke 20ms (50Hz sample rate)
    // Dit is snel genoeg om loopbewegingen (0.5-3 Hz) te detecteren
    static unsigned long lastAccelRead = 0;
    static uint32_t lastStepCount = 0;  // Track vorige stepCount voor detectie
    if (imuAvailable && stepCountingEnabled && (now - lastAccelRead >= 20)) {
        lastAccelRead = now;

        // Lees accelerometer waarden van QMI8658
        readAccelerometer();

        // Detecteer stappen gebaseerd op peak-valley algoritme
        // Deze functie update automatisch stepCount als een stap gedetecteerd wordt
        detectStep();

        // Check of er een nieuwe stap gedetecteerd is door stepCount te vergelijken
        if (stepCount != lastStepCount && !displayAsleep) {
            lastStepCount = stepCount;
            updateStepDisplay();
        }
    }

    // Periodieke display update voor stappen (elke 500ms)
    // Zorgt ervoor dat display gesynchroniseerd blijft
    // Update sneller dan voorheen voor betere responsiviteit
    static unsigned long lastStepDisplayUpdate = 0;
    if (!displayAsleep && (now - lastStepDisplayUpdate >= 500)) {
        lastStepDisplayUpdate = now;
        updateStepDisplay();
    }

    // Korte delay om CPU tijd vrij te geven
    // Voorkomt dat loop 100% CPU gebruikt
    // 5ms is kort genoeg voor responsive UI
    delay(5);
}

// ============================================================================
// AUDIO LIBRARY CALLBACKS
// ============================================================================
// Deze callbacks worden aangeroepen door de ESP32-audioI2S library
// voor debug informatie en status updates

// Wordt aangeroepen met algemene audio info
void audio_info(const char *info) {
    Serial.printf("[Audio-Lib] Info: %s\n", info);
}

// Wordt aangeroepen bij fouten
void audio_error(const char *info) {
    Serial.printf("[Audio-Lib] ERROR: %s\n", info);
}

// Wordt aangeroepen als bestand klaar is met afspelen
void audio_eof_mp3(const char *info) {
    Serial.printf("[Audio-Lib] Einde MP3: %s\n", info);
    audioPlaying = false;
    // Zet PA uit na afspelen
    digitalWrite(PA_CTRL, LOW);
}

// ID3 metadata
void audio_id3data(const char *info) {
    Serial.printf("[Audio-Lib] ID3: %s\n", info);
}

// ============================================================================
// EINDE SCRIPT
// ============================================================================
