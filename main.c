//*****************************************************************************
// EEC 172 Final Project - Smart Package Black Box
// Based on the user's Lab 4 AWS + OLED + BMA222 project.
//
// Features added:
//   1. Calibration baseline from BMA222 X/Y/Z
//   2. Tilt / Shock / Possible Drop classification
//   3. Severity level: LOW / MEDIUM / HIGH
//   4. Finite state machine: DISARMED -> CALIBRATING -> MONITORING -> WARNING -> ALERT_SENT
//   5. Auto calibration at startup, SW2 cancel/reset
//   6. SW2: cancel countdown / reset after alert
//   7. UART event log + recent event history
//   8. Dynamic AWS JSON payload for event type, severity, count, raw sensor values, IP, and GPS
//*****************************************************************************

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// SimpleLink
#include "simplelink.h"

// Driverlib
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"
#include "gpio.h"
#include "spi.h"

// Common interface
#include "uart_if.h"
#include "i2c_if.h"
#include "gpio_if.h"
#include "common.h"
#include "pinmux.h"

// OLED
#include "Adafruit_SSD1351.h"
#include "Adafruit_GFX.h"
#include "oled_test.h"

// AWS networking helpers
#include "utils/network_utils.h"

//*****************************************************************************
// ===== UPDATE THIS BEFORE EVERY RUN! =====
// TLS handshake will fail if these are more than ~30 minutes off.
//*****************************************************************************
#define DATE    17
#define MONTH    5
#define YEAR  2026
#define HOUR    20
#define MINUTE  30
#define SECOND   0

//*****************************************************************************
// AWS endpoint
//*****************************************************************************
#define APPLICATION_NAME      "SSL"
#define APPLICATION_VERSION   "SQ24"

#define SERVER_NAME           "a1y2xob7847evk-ats.iot.us-east-2.amazonaws.com"
#define GOOGLE_DST_PORT       8443
#define POSTHEADER  "POST /things/3200/shadow HTTP/1.1\r\n"
#define HOSTHEADER  "Host: a1y2xob7847evk-ats.iot.us-east-2.amazonaws.com\r\n"
#define CHEADER     "Connection: close\r\n"
#define CTHEADER    "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1   "Content-Length: "
#define CLHEADER2   "\r\n\r\n"

//*****************************************************************************
// Hardware constants
//*****************************************************************************
#define FOREVER 1

#define ACCEL_ADDR     0x18
#define ACCEL_REG_X    0x03
#define ACCEL_REG_Y    0x05
#define ACCEL_REG_Z    0x07
#define OLED_SPI_RATE  100000

#define SCREEN_W       128
#define SCREEN_H       128

// CC3200 LaunchPad button, configured in pinmux.c
// NOTE: SW3 is disabled in this version because UART0 GPS uses the UART RX/TX path.
// SW2 is still used for countdown cancel and reset after alert.
#define SW2_GPIO_BASE  GPIOA2_BASE
#define SW2_GPIO_PIN   0x40       // PIN_15, GPIO22, high when pressed

// Buzzer on PIN_45.
// Wire DFRobot DFR0032 SIG/S -> PIN_45, VCC/+ -> 3.3V, GND/- -> GND.
#define BUZZER_GPIO_BASE  GPIOA3_BASE
#define BUZZER_GPIO_PIN   0x80

// GPS UART0 settings. This matches the GPS OLED test that already showed SEEN=YES and FIX=YES.
#define GPS_UART_BASE    UARTA0_BASE
#define GPS_UART_CLK     PRCM_UARTA0
#define GPS_BAUD_RATE    9600
#define GPS_LINE_SIZE    128

// Delay constants. UtilsDelay is about 3 CPU cycles per count.
// On the CC3200 80 MHz MCU, this is approximately 100 ms.
#define DELAY_100MS    2666666

//*****************************************************************************
// Detection tuning constants
// You should tune these numbers by watching UART output on your real board.
//*****************************************************************************
#define CAL_SAMPLES              32
#define CONFIRM_SAMPLES           3
#define NORMAL_CLEAR_SAMPLES      8
#define DROP_WINDOW_SAMPLES      12

#define TILT_LOW                 22
#define TILT_MEDIUM              40
#define TILT_HIGH                58

#define SHOCK_MEDIUM             48
#define SHOCK_HIGH               85

#define DROP_FREEFALL_SUM        18

#define COUNTDOWN_SECONDS         3
#define HISTORY_SIZE              8

#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
// Types
//*****************************************************************************
typedef enum {
    STATE_BOOT = 0,
    STATE_DISARMED,
    STATE_CALIBRATING,
    STATE_MONITORING,
    STATE_EVENT_DISPLAY,
    STATE_COUNTDOWN,
    STATE_ALERT_SENT
} SystemState;

typedef enum {
    EVENT_NONE = 0,
    EVENT_TILT,
    EVENT_SHOCK,
    EVENT_DROP,
    EVENT_MANUAL_SOS
} EventType;

typedef enum {
    SEV_NONE = 0,
    SEV_LOW,
    SEV_MEDIUM,
    SEV_HIGH
} Severity;

typedef struct {
    int id;
    EventType type;
    Severity severity;
    signed char x;
    signed char y;
    signed char z;
    int tilt_score;
    int shock_score;
    int accel_sum;
    unsigned long time_ms;
    int sent_to_cloud;
} EventRecord;

//*****************************************************************************
// Global state
//*****************************************************************************
static SystemState g_state = STATE_BOOT;

static signed char g_base_x = 0;
static signed char g_base_y = 0;
static signed char g_base_z = 0;

static signed char g_last_x = 0;
static signed char g_last_y = 0;
static signed char g_last_z = 0;

static int g_drop_armed = 0;
static int g_pending_count = 0;
static EventType g_pending_type = EVENT_NONE;

static int g_event_count = 0;
static EventRecord g_last_event;
static EventRecord g_history[HISTORY_SIZE];

static int g_sw2_last = 0;
static unsigned long g_ms = 0;

// GPS state from UART0 NMEA sentences.
static char g_gps_line[GPS_LINE_SIZE];
static int g_gps_idx = 0;
static int g_gps_seen = 0;
static int g_gps_fix = 0;
static char g_gps_lat[20] = "0.000000";
static char g_gps_lon[20] = "0.000000";

//*****************************************************************************
// Prototypes
//*****************************************************************************
static void BoardInit(void);
static int set_time(void);
static void init_oled(void);
static int http_post_event(int iTLSSockID, EventRecord *ev);
static void ipv4_to_string(unsigned long ip, char *out);
static void init_gps_uart(void);
static void gps_poll_uart(void);
static int gps_location_ready(void);
static void smart_package_loop(void);

static void delay_100ms(void);
static void oled_line(int y, const char *s, unsigned int color, unsigned int bg, int size);
static void draw_disarmed_screen(void);
static void draw_calibrating_screen(void);
static void draw_monitoring_screen(signed char x, signed char y, signed char z);
static void draw_event_screen(EventRecord *ev, const char *action);
static void draw_countdown_screen(EventRecord *ev, int seconds_left);
static void draw_alert_sent_screen(EventRecord *ev, int cloud_ok);

static int sw2_pressed_once(void);
static void buzzer_on(void);
static void buzzer_off(void);
static void calibrate_baseline(void);
static EventType classify_motion(signed char ax, signed char ay, signed char az,
                                 int *tilt_score, int *shock_score, int *accel_sum);
static Severity estimate_severity(EventType type, int tilt_score, int shock_score, int accel_sum);
static void create_event(EventType type, Severity severity, signed char x, signed char y, signed char z,
                         int tilt_score, int shock_score, int accel_sum);
static void log_event_uart(EventRecord *ev);
static void print_history_uart(void);
static void wait_until_motion_clears(void);

static const char *event_name(EventType type);
static const char *severity_name(Severity severity);

signed char ReadAccel(unsigned char reg);

//*****************************************************************************
// Board init
//*****************************************************************************
static void BoardInit(void)
{
#ifndef USE_TIRTOS
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);
    PRCMCC3200MCUInit();
}

//*****************************************************************************
// Set device date/time so TLS cert validation passes.
//*****************************************************************************
static int set_time(void)
{
    long retVal;

    g_time.tm_day  = DATE;
    g_time.tm_mon  = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_hour = HOUR;
    g_time.tm_min  = MINUTE;
    g_time.tm_sec  = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                       SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                       sizeof(SlDateTime), (unsigned char *)(&g_time));
    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//*****************************************************************************
// Read one accel register from BMA222
//*****************************************************************************
signed char ReadAccel(unsigned char reg)
{
    unsigned char val = 0;
    I2C_IF_Write(ACCEL_ADDR, &reg, 1, 0);
    I2C_IF_Read(ACCEL_ADDR, &val, 1);
    return (signed char)val;
}

//*****************************************************************************
// Small timing helper
//*****************************************************************************
static void delay_100ms(void)
{
    MAP_UtilsDelay(DELAY_100MS);
    g_ms += 100;
}

//*****************************************************************************
// OLED init
//*****************************************************************************
static void init_oled(void)
{
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_GSPI);
    MAP_SPIReset(GSPI_BASE);

    MAP_SPIConfigSetExpClk(GSPI_BASE,
                           MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                           OLED_SPI_RATE,
                           SPI_MODE_MASTER,
                           SPI_SUB_MODE_0,
                           (SPI_3PIN_MODE | SPI_TURBO_OFF | SPI_WL_8));

    MAP_SPIEnable(GSPI_BASE);

    MAP_GPIOPinWrite(GPIOA0_BASE, 0x80, 0x80);   // CS high
    MAP_GPIOPinWrite(GPIOA0_BASE, 0x40, 0x00);   // DC low
    MAP_GPIOPinWrite(GPIOA3_BASE, 0x10, 0x10);   // RST high
    MAP_UtilsDelay(8000000);

    Adafruit_Init();
    fillScreen(BLACK);
}

//*****************************************************************************
// OLED text helpers
//*****************************************************************************
static void oled_line(int y, const char *s, unsigned int color, unsigned int bg, int size)
{
    int i;
    int x = 2;
    int step = 6 * size;

    for (i = 0; s[i] != '\0' && x < SCREEN_W - step; i++) {
        drawChar(x, y, s[i], color, bg, size);
        x += step;
    }
}

static void draw_disarmed_screen(void)
{
    fillScreen(BLACK);
    oled_line(4,  "SMART PACKAGE", CYAN, BLACK, 1);
    oled_line(20, "BLACK BOX", CYAN, BLACK, 2);
    oled_line(48, "State: DISARMED", WHITE, BLACK, 1);
    oled_line(64, "Auto calibrate", YELLOW, BLACK, 1);
    oled_line(78, "starting...", YELLOW, BLACK, 1);
    oled_line(100,"GPS enabled", WHITE, BLACK, 1);
}

static void draw_calibrating_screen(void)
{
    fillScreen(BLACK);
    oled_line(20, "CALIBRATING", YELLOW, BLACK, 2);
    oled_line(54, "Keep package", WHITE, BLACK, 1);
    oled_line(68, "stable...", WHITE, BLACK, 1);
}

static void draw_monitoring_screen(signed char x, signed char y, signed char z)
{
    char line[32];

    fillScreen(BLACK);
    oled_line(4, "MONITORING", GREEN, BLACK, 2);
    sprintf(line, "X:%4d", x);
    oled_line(34, line, WHITE, BLACK, 1);
    sprintf(line, "Y:%4d", y);
    oled_line(48, line, WHITE, BLACK, 1);
    sprintf(line, "Z:%4d", z);
    oled_line(62, line, WHITE, BLACK, 1);
    sprintf(line, "Events:%d", g_event_count);
    oled_line(80, line, CYAN, BLACK, 1);
    sprintf(line, "GPS:%s FIX:%s", g_gps_seen ? "Y" : "N", g_gps_fix ? "Y" : "N");
    oled_line(104, line, YELLOW, BLACK, 1);
}

static void draw_event_screen(EventRecord *ev, const char *action)
{
    char line[32];
    unsigned int bg;

    bg = (ev->severity == SEV_HIGH) ? RED : BLACK;
    fillScreen(bg);

    oled_line(2, "EVENT DETECTED", YELLOW, bg, 1);
    sprintf(line, "%s", event_name(ev->type));
    oled_line(22, line, WHITE, bg, 2);
    sprintf(line, "Severity:%s", severity_name(ev->severity));
    oled_line(52, line, WHITE, bg, 1);
    sprintf(line, "Count:%d", ev->id);
    oled_line(66, line, WHITE, bg, 1);
    sprintf(line, "X:%d Y:%d", ev->x, ev->y);
    oled_line(82, line, WHITE, bg, 1);
    sprintf(line, "Z:%d", ev->z);
    oled_line(96, line, WHITE, bg, 1);
    oled_line(112, action, CYAN, bg, 1);
}

static void draw_countdown_screen(EventRecord *ev, int seconds_left)
{
    char line[40];

    (void)seconds_left;

    fillScreen(RED);
    oled_line(4, "WARNING", WHITE, RED, 2);

    sprintf(line, "%s %s", event_name(ev->type), severity_name(ev->severity));
    oled_line(34, line, WHITE, RED, 1);

    sprintf(line, "Seen:%s Fix:%s",
            g_gps_seen ? "YES" : "NO",
            g_gps_fix ? "YES" : "NO");
    oled_line(52, line, YELLOW, RED, 1);

    if (gps_location_ready()) {
        oled_line(68, "GPS READY", GREEN, RED, 1);
        oled_line(84, "Sending alert", WHITE, RED, 1);
    } else {
        oled_line(68, "Waiting GPS fix", WHITE, RED, 1);
        oled_line(84, "Need seen+fix", WHITE, RED, 1);
        oled_line(104, "SW2 cancel", WHITE, RED, 1);
    }
}

static void draw_alert_sent_screen(EventRecord *ev, int cloud_ok)
{
    char line[32];

    fillScreen(BLACK);
    oled_line(8, "ALERT SENT", RED, BLACK, 2);
    sprintf(line, "%s %s", event_name(ev->type), severity_name(ev->severity));
    oled_line(44, line, WHITE, BLACK, 1);

    if (cloud_ok) {
        oled_line(66, "AWS POST: OK", GREEN, BLACK, 1);
    } else {
        oled_line(66, "AWS POST: FAIL", YELLOW, BLACK, 1);
    }

    sprintf(line, "Event #%d", ev->id);
    oled_line(84, line, WHITE, BLACK, 1);
    oled_line(106, "SW2 reset", CYAN, BLACK, 1);
}

//*****************************************************************************
// Button helpers. Buttons are active high on CC3200 LaunchPad.
//*****************************************************************************
static int sw2_pressed_once(void)
{
    int now = (MAP_GPIOPinRead(SW2_GPIO_BASE, SW2_GPIO_PIN) & SW2_GPIO_PIN) ? 1 : 0;
    int pressed = (now && !g_sw2_last);
    g_sw2_last = now;

    if (pressed) {
        MAP_UtilsDelay(800000);  // debounce
    }
    return pressed;
}


//*****************************************************************************
// Buzzer helpers. DFRobot Gravity Digital Buzzer Module DFR0032.
// Active HIGH: write GPIO high to sound, low to stop.
//*****************************************************************************
static void buzzer_on(void)
{
    MAP_GPIOPinWrite(BUZZER_GPIO_BASE, BUZZER_GPIO_PIN, BUZZER_GPIO_PIN);
}

static void buzzer_off(void)
{
    MAP_GPIOPinWrite(BUZZER_GPIO_BASE, BUZZER_GPIO_PIN, 0x00);
}

//*****************************************************************************
// Event strings
//*****************************************************************************
static const char *event_name(EventType type)
{
    switch (type) {
        case EVENT_TILT:       return "TILT";
        case EVENT_SHOCK:      return "SHOCK";
        case EVENT_DROP:       return "DROP";
        case EVENT_MANUAL_SOS: return "MANUAL";
        default:               return "NONE";
    }
}

static const char *severity_name(Severity severity)
{
    switch (severity) {
        case SEV_LOW:    return "LOW";
        case SEV_MEDIUM: return "MED";
        case SEV_HIGH:   return "HIGH";
        default:         return "NONE";
    }
}

//*****************************************************************************
// Calibration
//*****************************************************************************
static void calibrate_baseline(void)
{
    int i;
    int sx = 0;
    int sy = 0;
    int sz = 0;

    draw_calibrating_screen();
    UART_PRINT("\n\rCalibration started. Keep the package stable.\n\r");

    for (i = 0; i < CAL_SAMPLES; i++) {
        sx += ReadAccel(ACCEL_REG_X);
        sy += ReadAccel(ACCEL_REG_Y);
        sz += ReadAccel(ACCEL_REG_Z);
        MAP_UtilsDelay(400000);
    }

    g_base_x = (signed char)(sx / CAL_SAMPLES);
    g_base_y = (signed char)(sy / CAL_SAMPLES);
    g_base_z = (signed char)(sz / CAL_SAMPLES);

    g_last_x = g_base_x;
    g_last_y = g_base_y;
    g_last_z = g_base_z;

    g_drop_armed = 0;
    g_pending_count = 0;
    g_pending_type = EVENT_NONE;

    UART_PRINT("Baseline X=%d Y=%d Z=%d\n\r", g_base_x, g_base_y, g_base_z);
}

//*****************************************************************************
// Classification logic
//*****************************************************************************
static EventType classify_motion(signed char ax, signed char ay, signed char az,
                                 int *tilt_score, int *shock_score, int *accel_sum)
{
    int dx = abs((int)ax - (int)g_base_x);
    int dy = abs((int)ay - (int)g_base_y);
    int dz = abs((int)az - (int)g_base_z);
    int sx = abs((int)ax - (int)g_last_x);
    int sy = abs((int)ay - (int)g_last_y);
    int sz = abs((int)az - (int)g_last_z);
    int sum = abs((int)ax) + abs((int)ay) + abs((int)az);
    int tilt;
    int shock;

    tilt = dx;
    if (dy > tilt) tilt = dy;
    if (dz > tilt) tilt = dz;

    shock = sx + sy + sz;

    *tilt_score = tilt;
    *shock_score = shock;
    *accel_sum = sum;

    // Free-fall style clue: total acceleration becomes very small.
    // We arm DROP first, then confirm it if a shock follows within a short window.
    if (sum < DROP_FREEFALL_SUM) {
        g_drop_armed = DROP_WINDOW_SAMPLES;
    }

    if (g_drop_armed > 0) {
        g_drop_armed--;
        if (shock > SHOCK_MEDIUM) {
            return EVENT_DROP;
        }
    }

    if (shock > SHOCK_MEDIUM) {
        return EVENT_SHOCK;
    }

    if (tilt > TILT_LOW) {
        return EVENT_TILT;
    }

    return EVENT_NONE;
}

static Severity estimate_severity(EventType type, int tilt_score, int shock_score, int accel_sum)
{
    if (type == EVENT_MANUAL_SOS) {
        return SEV_HIGH;
    }

    if (type == EVENT_DROP) {
        if (shock_score > SHOCK_HIGH || accel_sum > 120) {
            return SEV_HIGH;
        }
        return SEV_MEDIUM;
    }

    if (type == EVENT_SHOCK) {
        if (shock_score > SHOCK_HIGH) {
            return SEV_HIGH;
        }
        return SEV_MEDIUM;
    }

    if (type == EVENT_TILT) {
        if (tilt_score > TILT_HIGH) {
            return SEV_HIGH;
        }
        if (tilt_score > TILT_MEDIUM) {
            return SEV_MEDIUM;
        }
        return SEV_LOW;
    }

    return SEV_NONE;
}

//*****************************************************************************
// Event creation, history, and UART log
//*****************************************************************************
static void create_event(EventType type, Severity severity, signed char x, signed char y, signed char z,
                         int tilt_score, int shock_score, int accel_sum)
{
    EventRecord ev;
    int index;

    g_event_count++;

    ev.id = g_event_count;
    ev.type = type;
    ev.severity = severity;
    ev.x = x;
    ev.y = y;
    ev.z = z;
    ev.tilt_score = tilt_score;
    ev.shock_score = shock_score;
    ev.accel_sum = accel_sum;
    ev.time_ms = g_ms;
    ev.sent_to_cloud = 0;

    g_last_event = ev;
    index = (g_event_count - 1) % HISTORY_SIZE;
    g_history[index] = ev;

    log_event_uart(&g_last_event);
}

static void log_event_uart(EventRecord *ev)
{
    UART_PRINT("\n\r===== PACKAGE EVENT LOG =====\n\r");
    UART_PRINT("Event ID: %d\n\r", ev->id);
    UART_PRINT("Type: %s\n\r", event_name(ev->type));
    UART_PRINT("Severity: %s\n\r", severity_name(ev->severity));
    UART_PRINT("Time: %lu ms\n\r", ev->time_ms);
    UART_PRINT("Raw Accel: X=%d Y=%d Z=%d\n\r", ev->x, ev->y, ev->z);
    UART_PRINT("Tilt score: %d\n\r", ev->tilt_score);
    UART_PRINT("Shock score: %d\n\r", ev->shock_score);
    UART_PRINT("Accel sum: %d\n\r", ev->accel_sum);
    UART_PRINT("=============================\n\r");
}

static void print_history_uart(void)
{
    int i;
    int count;
    int start;
    int idx;
    EventRecord *ev;

    UART_PRINT("\n\r===== RECENT EVENT HISTORY =====\n\r");

    count = (g_event_count < HISTORY_SIZE) ? g_event_count : HISTORY_SIZE;
    start = g_event_count - count;

    if (count == 0) {
        UART_PRINT("No events recorded yet.\n\r");
    }

    for (i = 0; i < count; i++) {
        idx = (start + i) % HISTORY_SIZE;
        ev = &g_history[idx];
        UART_PRINT("#%d  %s  %s  X=%d Y=%d Z=%d  t=%lu ms\n\r",
                   ev->id, event_name(ev->type), severity_name(ev->severity),
                   ev->x, ev->y, ev->z, ev->time_ms);
    }

    UART_PRINT("===============================\n\r");
}

//*****************************************************************************
// After a low event or canceled warning, wait for stable readings.
// This prevents one physical motion from being counted many times.
//*****************************************************************************
static void wait_until_motion_clears(void)
{
    int stable = 0;
    signed char ax, ay, az;
    int tilt_score, shock_score, accel_sum;
    EventType t;

    while (stable < NORMAL_CLEAR_SAMPLES) {
        ax = ReadAccel(ACCEL_REG_X);
        ay = ReadAccel(ACCEL_REG_Y);
        az = ReadAccel(ACCEL_REG_Z);

        t = classify_motion(ax, ay, az, &tilt_score, &shock_score, &accel_sum);

        g_last_x = ax;
        g_last_y = ay;
        g_last_z = az;

        if (t == EVENT_NONE) {
            stable++;
        } else {
            stable = 0;
        }

        delay_100ms();
    }
}
static void ipv4_to_string(unsigned long ip, char *out)
{
    sprintf(out, "%d.%d.%d.%d",
            SL_IPV4_BYTE(ip, 3), SL_IPV4_BYTE(ip, 2),
            SL_IPV4_BYTE(ip, 1), SL_IPV4_BYTE(ip, 0));
}

//*****************************************************************************
// GPS helpers. GPS outputs NMEA sentences over UART0 at 9600 baud.
//*****************************************************************************
static void init_gps_uart(void)
{
    MAP_PRCMPeripheralClkEnable(GPS_UART_CLK, PRCM_RUN_MODE_CLK);
    MAP_UARTConfigSetExpClk(GPS_UART_BASE,
                            MAP_PRCMPeripheralClockGet(GPS_UART_CLK),
                            GPS_BAUD_RATE,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    MAP_UARTFIFOEnable(GPS_UART_BASE);
    MAP_UARTEnable(GPS_UART_BASE);
}

static int split_csv(char *s, char *fields[], int max_fields)
{
    int count = 0;
    fields[count++] = s;
    while (*s && count < max_fields) {
        if (*s == ',') {
            *s = '\0';
            fields[count++] = s + 1;
        }
        s++;
    }
    return count;
}

static void nmea_to_decimal_string(const char *raw, const char *dir, char *out)
{
    double v;
    int deg;
    double minutes;
    double dec;
    int neg;
    unsigned long scaled;
    unsigned long whole;
    unsigned long frac;

    if (raw == NULL || raw[0] == '\0') {
        strcpy(out, "0.000000");
        return;
    }

    v = atof(raw);
    deg = (int)(v / 100.0);
    minutes = v - ((double)deg * 100.0);
    dec = (double)deg + (minutes / 60.0);

    if (dir != NULL && (dir[0] == 'S' || dir[0] == 'W')) {
        dec = -dec;
    }

    neg = (dec < 0.0) ? 1 : 0;
    if (neg) {
        dec = -dec;
    }

    scaled = (unsigned long)(dec * 1000000.0 + 0.5);
    whole = scaled / 1000000UL;
    frac = scaled % 1000000UL;

    sprintf(out, "%s%lu.%06lu", neg ? "-" : "", whole, frac);
}

static void parse_nmea_line(char *line)
{
    char buf[GPS_LINE_SIZE];
    char *fields[24];
    int n;

    if (line[0] != '$') {
        return;
    }

    g_gps_seen = 1;

    strncpy(buf, line, GPS_LINE_SIZE - 1);
    buf[GPS_LINE_SIZE - 1] = '\0';

    n = split_csv(buf, fields, 24);
    if (n <= 0) {
        return;
    }

    if (strncmp(fields[0], "$GPGGA", 6) == 0 || strncmp(fields[0], "$GNGGA", 6) == 0) {
        if (n > 6 && fields[6][0] != '0' && fields[6][0] != '\0') {
            g_gps_fix = 1;
            nmea_to_decimal_string(fields[2], fields[3], g_gps_lat);
            nmea_to_decimal_string(fields[4], fields[5], g_gps_lon);
        }
    }
    else if (strncmp(fields[0], "$GPRMC", 6) == 0 || strncmp(fields[0], "$GNRMC", 6) == 0) {
        if (n > 6 && fields[2][0] == 'A') {
            g_gps_fix = 1;
            nmea_to_decimal_string(fields[3], fields[4], g_gps_lat);
            nmea_to_decimal_string(fields[5], fields[6], g_gps_lon);
        }
    }
}

static void gps_poll_uart(void)
{
    char c;

    while (MAP_UARTCharsAvail(GPS_UART_BASE)) {
        c = (char)MAP_UARTCharGetNonBlocking(GPS_UART_BASE);

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            g_gps_line[g_gps_idx] = '\0';
            parse_nmea_line(g_gps_line);
            g_gps_idx = 0;
        } else {
            if (g_gps_idx < GPS_LINE_SIZE - 1) {
                g_gps_line[g_gps_idx++] = c;
            } else {
                g_gps_idx = 0;
            }
        }
    }
}

static int gps_location_ready(void)
{
    return (g_gps_seen && g_gps_fix);
}

//*****************************************************************************
// HTTP POST to AWS IoT shadow over the supplied TLS socket
//*****************************************************************************
static int http_post_event(int iTLSSockID, EventRecord *ev)
{
    char acSendBuff[1600];
    char acRecvbuff[1460];
    char cCLLength[32];
    char data[960];
    char device_ip[20];
    char gateway_ip[20];
    char *pcBufHeaders;
    int lRetVal = 0;
    int dataLength;

    ipv4_to_string(g_ulDeviceIP, device_ip);
    ipv4_to_string(g_ulGatewayIP, gateway_ip);

    gps_poll_uart();

    sprintf(data,
            "{\"state\":{\"desired\":{" \
            "\"device_id\":\"cc3200_package_01\"," \
            "\"product\":\"Smart Package Black Box\"," \
            "\"event_type\":\"%s\"," \
            "\"severity\":\"%s\"," \
            "\"event_count\":%d," \
            "\"x\":%d,\"y\":%d,\"z\":%d," \
            "\"tilt_score\":%d," \
            "\"shock_score\":%d," \
            "\"accel_sum\":%d," \
            "\"time_ms\":%lu," \
            "\"device_local_ip\":\"%s\"," \
            "\"gateway_ip\":\"%s\"," \
            "\"gps_seen\":%d," \
            "\"gps_fix\":%d," \
            "\"gps_lat\":\"%s\"," \
            "\"gps_lon\":\"%s\"," \
            "\"location_source\":\"GPS coordinates from Adafruit Ultimate GPS\"," \
            "\"location_accuracy\":\"GPS fix required for valid coordinates\"," \
            "\"status\":\"alert_sent\"}}}\r\n\r\n",
            event_name(ev->type), severity_name(ev->severity), ev->id,
            ev->x, ev->y, ev->z,
            ev->tilt_score, ev->shock_score, ev->accel_sum,
            ev->time_ms, device_ip, gateway_ip,
            g_gps_seen, g_gps_fix, g_gps_lat, g_gps_lon);

    dataLength = strlen(data);

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);  pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);  pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);     pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, CTHEADER);    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);   pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);
    strcpy(pcBufHeaders, cCLLength);   pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);   pcBufHeaders += strlen(CLHEADER2);
    strcpy(pcBufHeaders, data);        pcBufHeaders += strlen(data);

    UART_PRINT("\n\r===== AWS POST PAYLOAD =====\n\r");
    UART_PRINT(data);
    UART_PRINT("\n\r============================\n\r");

    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if (lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r", lRetVal);
        sl_Close(iTLSSockID);
        return lRetVal;
    }

    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff) - 2, 0);
    if (lRetVal < 0) {
        UART_PRINT("Recv failed. Error Number: %i\n\r", lRetVal);
        sl_Close(iTLSSockID);
        return lRetVal;
    }

    acRecvbuff[lRetVal] = '\0';
    UART_PRINT(acRecvbuff);
    UART_PRINT("\n\r\n\r");
    sl_Close(iTLSSockID);
    return 0;
}

//*****************************************************************************
// Main finite-state monitoring loop
//*****************************************************************************
static void smart_package_loop(void)
{
    signed char ax, ay, az;
    int tilt_score, shock_score, accel_sum;
    EventType candidate;
    Severity severity;
    int monitor_redraw = 0;
    int seconds_left;
    int tick;
    int canceled;
    int lRetVal;
    int cloud_ok;

    // GPS uses UART0, so SW3 is disabled. Auto-calibrate at startup.
    g_state = STATE_CALIBRATING;
    draw_calibrating_screen();

    while (FOREVER) {
        if (g_state == STATE_DISARMED) {
            gps_poll_uart();
            if (sw2_pressed_once()) {
                print_history_uart();
            }
            delay_100ms();
        }
        else if (g_state == STATE_CALIBRATING) {
            calibrate_baseline();
            g_state = STATE_MONITORING;
            monitor_redraw = 0;
        }
        else if (g_state == STATE_MONITORING) {
            gps_poll_uart();
            ax = ReadAccel(ACCEL_REG_X);
            ay = ReadAccel(ACCEL_REG_Y);
            az = ReadAccel(ACCEL_REG_Z);

            candidate = classify_motion(ax, ay, az, &tilt_score, &shock_score, &accel_sum);

            if (candidate != EVENT_NONE) {
                if (candidate == g_pending_type) {
                    g_pending_count++;
                } else {
                    g_pending_type = candidate;
                    g_pending_count = 1;
                }

                if (g_pending_count >= CONFIRM_SAMPLES) {
                    severity = estimate_severity(candidate, tilt_score, shock_score, accel_sum);
                    create_event(candidate, severity, ax, ay, az, tilt_score, shock_score, accel_sum);

                    /*
                     * Every confirmed event now enters WARNING/GPS-WAIT.
                     * Buzzer sounds immediately when WARNING behavior is detected.
                     * The system does not send AWS until GPS is seen and fixed.
                     */
                    buzzer_on();
                    g_state = STATE_COUNTDOWN;

                    g_pending_count = 0;
                    g_pending_type = EVENT_NONE;
                }
            } else {
                g_pending_count = 0;
                g_pending_type = EVENT_NONE;
            }

            g_last_x = ax;
            g_last_y = ay;
            g_last_z = az;

            if (monitor_redraw == 0 && g_state == STATE_MONITORING) {
                draw_monitoring_screen(ax, ay, az);
            }
            monitor_redraw = (monitor_redraw + 1) % 10;

            delay_100ms();
        }
        else if (g_state == STATE_EVENT_DISPLAY) {
            draw_event_screen(&g_last_event, "LOW: logged only");
            UART_PRINT("Low severity event logged. No cloud alert sent.\n\r");
            MAP_UtilsDelay(DELAY_100MS * 10);
            g_ms += 1000;
            wait_until_motion_clears();
            g_state = STATE_MONITORING;
            monitor_redraw = 0;
        }
        else if (g_state == STATE_COUNTDOWN) {
            canceled = 0;
            tick = 0;

            /*
             * No countdown here. Stay on the red WARNING screen and keep
             * refreshing GPS status until gps_seen=1 and gps_fix=1 with
             * non-zero coordinates. Then send the AWS alert.
             */
            while (!gps_location_ready()) {
                gps_poll_uart();

                if ((tick % 10) == 0) {
                    draw_countdown_screen(&g_last_event, 0);
                }

                if (sw2_pressed_once()) {
                    canceled = 1;
                    break;
                }

                delay_100ms();
                tick++;
            }

            if (canceled) {
                buzzer_off();
                UART_PRINT("Alert canceled by SW2. Event was logged but not sent.\n\r");
                draw_event_screen(&g_last_event, "CANCELED by SW2");
                MAP_UtilsDelay(DELAY_100MS * 10);
                g_ms += 1000;
                wait_until_motion_clears();
                g_state = STATE_MONITORING;
                monitor_redraw = 0;
            } else {
                buzzer_off();
                draw_countdown_screen(&g_last_event, 0);
                g_state = STATE_ALERT_SENT;
            }
        }
        else if (g_state == STATE_ALERT_SENT) {
            gps_poll_uart();
            UART_PRINT("Opening TLS socket for AWS alert...\n\r");
            lRetVal = tls_connect();
            cloud_ok = 0;

            if (lRetVal < 0) {
                UART_PRINT("TLS connect failed (%d). Alert remains local/UART only.\n\r", lRetVal);
            } else {
                lRetVal = http_post_event(lRetVal, &g_last_event);
                if (lRetVal == 0) {
                    cloud_ok = 1;
                    g_last_event.sent_to_cloud = 1;
                    g_history[(g_last_event.id - 1) % HISTORY_SIZE].sent_to_cloud = 1;
                }
            }

            draw_alert_sent_screen(&g_last_event, cloud_ok);
            UART_PRINT("Press SW2 to reset back to monitoring.\n\r");

            while (!sw2_pressed_once()) {
                delay_100ms();
            }

            wait_until_motion_clears();
            g_state = STATE_MONITORING;
            monitor_redraw = 0;
        }
    }
}

//*****************************************************************************
// Main
//*****************************************************************************
void main(void)
{
    long lRetVal;

    BoardInit();
    PinMuxConfig();

    InitTerm();
    ClearTerm();
    UART_PRINT("\n\r=== Smart Package Black Box + AWS IoT + GPS ===\n\r");

    // I2C for on-board BMA222 accelerometer
    I2C_IF_Open(I2C_MASTER_MODE_FST);

    // OLED
    init_oled();
    UART_PRINT("OLED ready.\n\r");

    // Bring up WiFi once at startup.
    g_app_config.host = (signed char *)SERVER_NAME;
    g_app_config.port = GOOGLE_DST_PORT;

    UART_PRINT("Connecting to WiFi...\n\r");
    lRetVal = connectToAccessPoint();

    if (lRetVal < 0) {
        UART_PRINT("WiFi connect failed (%d). Running local mode only.\n\r", lRetVal);
    } else {
        lRetVal = set_time();
        if (lRetVal < 0) {
            UART_PRINT("set_time failed (%d). AWS may fail.\n\r", lRetVal);
        }
    }

    /*
     * connectToAccessPoint() may reconfigure pins.
     * Re-apply pinmux, then configure UART0 on PIN3/PIN4 for GPS.
     * After init_gps_uart(), UART0 is used by GPS at 9600 baud, so
     * do not rely on UART_PRINT for debugging. Use OLED and AWS email.
     */
    PinMuxConfig();
    buzzer_off();
    init_gps_uart();

    smart_package_loop();
}
