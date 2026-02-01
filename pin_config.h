#pragma once

#define XPOWERS_CHIP_AXP2101

#define LCD_SDIO0 4
#define LCD_SDIO1 5
#define LCD_SDIO2 6
#define LCD_SDIO3 7
#define LCD_SCLK 11
#define LCD_CS 12
#define LCD_RESET 8
#define LCD_WIDTH 410
#define LCD_HEIGHT 502

// TOUCH
#define IIC_SDA 15
#define IIC_SCL 14
#define TP_INT 38
#define TP_RESET 9

// SD
const int SDMMC_CLK = 2;
const int SDMMC_CMD = 1;
const int SDMMC_DATA = 3;
const int SDMMC_CS = 17;

// AUDIO - ES8311 Codec (I2C + I2S)
#define ES8311_I2C_ADDR   0x18    // ES8311 I2C adres
#define I2S_MCLK          16      // Master Clock
#define I2S_BCLK          41      // Bit Clock (SCLK)
#define I2S_LRCK          45      // Left/Right Clock (Word Select)
#define I2S_DOUT          40      // Data OUT naar speaker (DSDIN)
#define I2S_DIN           42      // Data IN van microfoon (ASDOUT)
#define PA_CTRL           46      // Power Amplifier Control (speaker enable)