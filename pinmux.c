#include "pinmux.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "pin.h"
#include "gpio.h"
#include "prcm.h"

//*****************************************************************************
// Pinmux for Smart Package Black Box with GPS on UART0 P1 header.
//
// GPS wiring that already worked in your OLED test:
//   GPS TX -> CC3200 UART0_RX path, P1 Ref 3, Dev Pin 4
//   GPS RX -> CC3200 UART0_TX path, P1 Ref 4, Dev Pin 3
//
// Important:
//   SW3 is NOT configured here because PIN_04 is used by UART0 RX for GPS.
//   The main program auto-calibrates at startup instead.
//   SW2 is still used for cancel countdown and reset after alert.
//*****************************************************************************
void PinMuxConfig(void)
{
    // Do NOT mark PIN_03 or PIN_04 as unused. They are UART0 GPS pins.
    // Do NOT mark PIN_15 as unused. It is SW2.
    // Do NOT mark PIN_45 as unused. It is buzzer output.
    PinModeSet(PIN_21, PIN_MODE_0);
    PinModeSet(PIN_52, PIN_MODE_0);
    PinModeSet(PIN_53, PIN_MODE_0);
    PinModeSet(PIN_58, PIN_MODE_0);
    PinModeSet(PIN_59, PIN_MODE_0);
    PinModeSet(PIN_60, PIN_MODE_0);
    PinModeSet(PIN_63, PIN_MODE_0);
    PinModeSet(PIN_64, PIN_MODE_0);

    // Clocks
    PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);   // GPS UART0
    PRCMPeripheralClkEnable(PRCM_I2CA0,  PRCM_RUN_MODE_CLK);   // BMA222 I2C
    PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK);   // OLED DC/CS
    PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK);   // SW2
    PRCMPeripheralClkEnable(PRCM_GPIOA3, PRCM_RUN_MODE_CLK);   // OLED RESET
    PRCMPeripheralClkEnable(PRCM_GSPI,   PRCM_RUN_MODE_CLK);   // OLED SPI

    // UART0 on P1 header. This is the configuration that made SEEN=YES/FIX=YES.
    // P1 Ref 4 = UART0_TX = Dev Pin 3, connect to GPS RX if used.
    // P1 Ref 3 = UART0_RX = Dev Pin 4, connect to GPS TX.
    PinTypeUART(PIN_03, PIN_MODE_7);   // UART0_TX
    PinTypeUART(PIN_04, PIN_MODE_7);   // UART0_RX

    // I2C for on-board BMA222 accelerometer
    PinTypeI2C(PIN_01, PIN_MODE_1);    // I2C_SCL
    PinTypeI2C(PIN_02, PIN_MODE_1);    // I2C_SDA

    // SW2 button: PIN_15, GPIO22, GPIOA2 bit 0x40
    // Used for cancel countdown and reset after alert.
    PinTypeGPIO(PIN_15, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA2_BASE, 0x40, GPIO_DIR_MODE_IN);

    // Buzzer output: PIN_45 / GPIO31 / GPIOA3 bit 0x80.
    // DFRobot DFR0032 SIG connects here. Start LOW so it is quiet at boot.
    PinTypeGPIO(PIN_45, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA3_BASE, 0x80, GPIO_DIR_MODE_OUT);
    GPIOPinWrite(GPIOA3_BASE, 0x80, 0x00);

    // OLED control pins
    PinTypeGPIO(PIN_61, PIN_MODE_0, false);   // DC
    GPIODirModeSet(GPIOA0_BASE, 0x40, GPIO_DIR_MODE_OUT);
    GPIOPinWrite(GPIOA0_BASE, 0x40, 0x00);

    PinTypeGPIO(PIN_62, PIN_MODE_0, false);   // CS
    GPIODirModeSet(GPIOA0_BASE, 0x80, GPIO_DIR_MODE_OUT);
    GPIOPinWrite(GPIOA0_BASE, 0x80, 0x80);

    PinTypeGPIO(PIN_18, PIN_MODE_0, false);   // RESET
    GPIODirModeSet(GPIOA3_BASE, 0x10, GPIO_DIR_MODE_OUT);
    GPIOPinWrite(GPIOA3_BASE, 0x10, 0x10);

    // OLED SPI
    PinTypeSPI(PIN_05, PIN_MODE_7);    // CLK
    PinTypeSPI(PIN_06, PIN_MODE_7);    // MISO, unused by OLED
    PinTypeSPI(PIN_07, PIN_MODE_7);    // MOSI
}

