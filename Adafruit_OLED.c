#include <string.h>
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "gpio.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"
#include "uart_if.h"
#include "pinmux.h"
#include "Adafruit_SSD1351.h"


#define OLED_DC_PORT   GPIOA0_BASE
#define OLED_DC_PIN    0x40

#define OLED_CS_PORT   GPIOA0_BASE
#define OLED_CS_PIN    0x80

#define OLED_RST_PORT  GPIOA3_BASE
#define OLED_RST_PIN   0x10

void drawFastVLine(int x, int y, int h, unsigned int color)
{
    unsigned int i;

    if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT))
        return;

    if (y + h > SSD1351HEIGHT)
        h = SSD1351HEIGHT - y - 1;

    if (h < 0) return;

    writeCommand(SSD1351_CMD_SETCOLUMN);
    writeData(x);
    writeData(x);
    writeCommand(SSD1351_CMD_SETROW);
    writeData(y);
    writeData(y + h - 1);
    writeCommand(SSD1351_CMD_WRITERAM);

    for (i = 0; i < h; i++) {
        writeData(color >> 8);
        writeData(color);
    }
}

void drawFastHLine(int x, int y, int w, unsigned int color)
{
    unsigned int i;

    if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT))
        return;

    if (x + w > SSD1351WIDTH)
        w = SSD1351WIDTH - x - 1;

    if (w < 0) return;

    writeCommand(SSD1351_CMD_SETCOLUMN);
    writeData(x);
    writeData(x + w - 1);
    writeCommand(SSD1351_CMD_SETROW);
    writeData(y);
    writeData(y);
    writeCommand(SSD1351_CMD_WRITERAM);

    for (i = 0; i < w; i++) {
        writeData(color >> 8);
        writeData(color);
    }
}

void drawPixel(int x, int y, unsigned int color)
{
    if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT)) return;
    if ((x < 0) || (y < 0)) return;

    goTo(x, y);
    writeData(color >> 8);
    writeData(color);
}
void writeCommand(unsigned char c)
{
    unsigned long dummy;
    MAP_GPIOPinWrite(OLED_DC_PORT, OLED_DC_PIN, 0);
    MAP_GPIOPinWrite(OLED_CS_PORT, OLED_CS_PIN, 0);
    MAP_SPIDataPut(GSPI_BASE, c);
    MAP_SPIDataGet(GSPI_BASE, &dummy);
    MAP_GPIOPinWrite(OLED_CS_PORT, OLED_CS_PIN, OLED_CS_PIN);
}

void writeData(unsigned char c)
{
    unsigned long dummy;
    MAP_GPIOPinWrite(OLED_DC_PORT, OLED_DC_PIN, OLED_DC_PIN);
    MAP_GPIOPinWrite(OLED_CS_PORT, OLED_CS_PIN, 0);
    MAP_SPIDataPut(GSPI_BASE, c);
    MAP_SPIDataGet(GSPI_BASE, &dummy);
    MAP_GPIOPinWrite(OLED_CS_PORT, OLED_CS_PIN, OLED_CS_PIN);
}

void Adafruit_Init(void)
{
    MAP_GPIOPinWrite(OLED_RST_PORT, OLED_RST_PIN, 0);
    MAP_UtilsDelay(8000000);
    MAP_GPIOPinWrite(OLED_RST_PORT, OLED_RST_PIN, OLED_RST_PIN);
    MAP_UtilsDelay(8000000);

    writeCommand(SSD1351_CMD_COMMANDLOCK);  writeData(0x12);
    writeCommand(SSD1351_CMD_COMMANDLOCK);  writeData(0xB1);
    writeCommand(SSD1351_CMD_DISPLAYOFF);
    writeCommand(SSD1351_CMD_CLOCKDIV);     writeData(0xF1);
    writeCommand(SSD1351_CMD_MUXRATIO);     writeData(127);
    writeCommand(SSD1351_CMD_SETREMAP);     writeData(0x74);

    writeCommand(SSD1351_CMD_SETCOLUMN);
    writeData(0x00); writeData(0x7F);

    writeCommand(SSD1351_CMD_SETROW);
    writeData(0x00); writeData(0x7F);

    writeCommand(SSD1351_CMD_STARTLINE);
    if (SSD1351HEIGHT == 96) writeData(96); else writeData(0);

    writeCommand(SSD1351_CMD_DISPLAYOFFSET);   writeData(0x00);
    writeCommand(SSD1351_CMD_SETGPIO);         writeData(0x00);
    writeCommand(SSD1351_CMD_FUNCTIONSELECT);  writeData(0x01);
    writeCommand(SSD1351_CMD_PRECHARGE);       writeData(0x32);
    writeCommand(SSD1351_CMD_VCOMH);           writeData(0x05);
    writeCommand(SSD1351_CMD_NORMALDISPLAY);

    writeCommand(SSD1351_CMD_CONTRASTABC);
    writeData(0xC8); writeData(0x80); writeData(0xC8);

    writeCommand(SSD1351_CMD_CONTRASTMASTER);  writeData(0x0F);

    writeCommand(SSD1351_CMD_SETVSL);
    writeData(0xA0); writeData(0xB5); writeData(0x55);

    writeCommand(SSD1351_CMD_PRECHARGE2);      writeData(0x01);
    writeCommand(SSD1351_CMD_DISPLAYON);
}

void goTo(int x, int y) {
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT)) return;
  writeCommand(SSD1351_CMD_SETCOLUMN);
  writeData(x); writeData(SSD1351WIDTH-1);
  writeCommand(SSD1351_CMD_SETROW);
  writeData(y); writeData(SSD1351HEIGHT-1);
  writeCommand(SSD1351_CMD_WRITERAM);
}

unsigned int Color565(unsigned char r, unsigned char g, unsigned char b) {
  unsigned int c;
  c = r >> 3; c <<= 6; c |= g >> 2; c <<= 5; c |= b >> 3;
  return c;
}

void fillScreen(unsigned int fillcolor) {
  fillRect(0, 0, SSD1351WIDTH, SSD1351HEIGHT, fillcolor);
}

void fillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int fillcolor)
{
  unsigned int i;
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT)) return;
  if (y + h > SSD1351HEIGHT) h = SSD1351HEIGHT - y - 1;
  if (x + w > SSD1351WIDTH) w = SSD1351WIDTH - x - 1;

  writeCommand(SSD1351_CMD_SETCOLUMN);
  writeData(x); writeData(x + w - 1);
  writeCommand(SSD1351_CMD_SETROW);
  writeData(y); writeData(y + h - 1);
  writeCommand(SSD1351_CMD_WRITERAM);

  for (i = 0; i < w * h; i++) {
    writeData(fillcolor >> 8);
    writeData(fillcolor);
  }
}


