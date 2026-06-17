# Smart Package Black Box with AWS IoT and GPS

An embedded IoT system that monitors package handling conditions, classifies package events such as tilt, shock, possible drop, and manual SOS, displays real-time status on an OLED screen, and sends cloud alerts to AWS IoT with GPS location data.

This project was built on a TI CC3200 LaunchPad and extends a basic AWS/OLED/BMA222 accelerometer demo into a more complete package-condition monitoring and evidence system.

## Overview

Package damage can happen during shipping, but it is often difficult to prove when or where the package was mishandled. This project acts as a **smart package black box**. It records motion events, classifies the type and severity of the event, provides local warning feedback, and sends structured event data to the cloud.

The system uses accelerometer readings to detect abnormal motion, an OLED display to show the current system state, a buzzer for local warning, GPS for location context, and AWS IoT Shadow over TLS for remote event reporting.

## Key Features

* Automatic startup calibration using baseline accelerometer readings
* Tilt, shock, possible drop, and manual SOS event classification
* Severity estimation with LOW, MEDIUM, and HIGH event levels
* Finite-state monitoring flow for calibration, monitoring, warning, GPS wait, and alert sent states
* OLED-based local user interface
* Buzzer warning output for confirmed events
* SW2 button support for canceling or resetting alerts
* Recent event history stored locally
* UART-based GPS parsing from NMEA sentences
* GPS fix check before sending cloud alerts
* Dynamic AWS IoT JSON payload with event type, severity, raw sensor values, scores, IP address, and GPS coordinates
* TLS-based HTTP POST to AWS IoT Shadow

## Hardware Used

* TI CC3200 LaunchPad
* On-board BMA222 accelerometer
* Adafruit / SSD1351-style OLED display
* Adafruit Ultimate GPS module or compatible UART GPS module
* DFRobot digital buzzer module or compatible GPIO buzzer
* Wi-Fi connection for AWS IoT communication

## Software and Tools

* C
* TI Code Composer Studio
* TI SimpleLink SDK / driverlib
* AWS IoT Shadow REST API over TLS
* UART, SPI, I2C, and GPIO
* NMEA GPS parsing
* Embedded finite-state-machine design

## System Architecture

The system follows a finite-state-machine design:

```text
BOOT
  -> CALIBRATING
  -> MONITORING
  -> WARNING / GPS WAIT
  -> ALERT_SENT
  -> MONITORING
```

During calibration, the system samples accelerometer values and computes a stable baseline. During monitoring, it continuously reads acceleration values and compares them with the baseline and previous samples. When an abnormal event is detected and confirmed across multiple samples, the system creates an event record, activates the buzzer, displays a warning screen, waits for a GPS fix, and sends the alert to AWS IoT.

## Event Classification Logic

The project classifies events using motion features derived from accelerometer readings.

### Tilt Detection

Tilt is detected by comparing the current X/Y/Z accelerometer values against the calibrated baseline. A larger deviation indicates stronger tilt.

### Shock Detection

Shock is detected by comparing the current accelerometer values against the previous sample. A large sudden change indicates impact or vibration.

### Possible Drop Detection

Possible drop detection uses a two-step rule. The system first looks for a low total acceleration pattern that may indicate free fall. If a strong shock follows within a short window, the event is classified as a possible drop.

### Severity Estimation

Each event is assigned a severity level based on tilt score, shock score, and acceleration sum.

```text
LOW     - Minor tilt or motion event
MEDIUM  - Strong tilt, shock, or possible drop
HIGH    - Severe shock, severe tilt, drop, or manual SOS
```

## AWS IoT Payload

When an alert is sent, the device generates a JSON payload similar to the following:

```json
{
  "state": {
    "desired": {
      "device_id": "cc3200_package_01",
      "product": "Smart Package Black Box",
      "event_type": "SHOCK",
      "severity": "HIGH",
      "event_count": 1,
      "x": 0,
      "y": 0,
      "z": 0,
      "tilt_score": 0,
      "shock_score": 0,
      "accel_sum": 0,
      "time_ms": 0,
      "device_local_ip": "0.0.0.0",
      "gateway_ip": "0.0.0.0",
      "gps_seen": 1,
      "gps_fix": 1,
      "gps_lat": "0.000000",
      "gps_lon": "0.000000",
      "location_source": "GPS coordinates from UART GPS module",
      "status": "alert_sent"
    }
  }
}
```

Sensitive credentials, certificates, Wi-Fi settings, and private AWS configuration should not be committed to a public repository.

## Repository Structure

```text
.
├── main.c                  # Main application logic, state machine, event classification, GPS, AWS POST
├── pinmux.c / pinmux.h     # CC3200 pin configuration for UART, SPI, I2C, GPIO, GPS, buzzer, OLED
├── oled_test.c / .h        # OLED display helpers
├── Adafruit_GFX.c / .h     # Graphics support for OLED drawing
├── Adafruit_OLED.c         # OLED driver support
├── utils/
│   ├── network_utils.c     # Wi-Fi, TLS, and network helper logic
│   └── network_utils.h
├── targetConfig/           # CCS target configuration
└── Release/                # Build output directory, ignored for source control
```

## Setup and Build

1. Open the project in **TI Code Composer Studio**.
2. Connect the TI CC3200 LaunchPad.
3. Configure Wi-Fi and AWS IoT settings in the project.
4. Make sure required certificates are flashed or available for TLS communication.
5. Update the system time constants before running, since TLS validation requires a correct date and time.
6. Build and flash the project to the CC3200.
7. Open the serial terminal or observe the OLED display for system status.

## Hardware Connections

### GPS

The GPS module communicates over UART0 at 9600 baud.

```text
GPS TX -> CC3200 UART0 RX
GPS RX -> CC3200 UART0 TX
GND    -> GND
VCC    -> 3.3V or the module's required input voltage
```

### Buzzer

```text
Buzzer SIG -> CC3200 GPIO output pin
Buzzer VCC -> 3.3V
Buzzer GND -> GND
```

### OLED

The OLED display uses SPI plus GPIO control pins for chip select, reset, and data/command control.

### Accelerometer

The project uses the on-board BMA222 accelerometer over I2C.

## Demo Flow

1. Power on the CC3200 board.
2. The system calibrates the accelerometer baseline.
3. The OLED displays monitoring status.
4. Tilt, shake, or drop the device to trigger an event.
5. The system classifies the event and estimates severity.
6. The buzzer activates and the OLED displays a warning.
7. The device waits for a GPS fix.
8. Once GPS is ready, the event is sent to AWS IoT.
9. The OLED displays whether the AWS POST succeeded.
10. Press SW2 to reset back to monitoring.

## My Contributions

* Extended a basic CC3200 AWS/OLED/accelerometer demo into a smart package monitoring system
* Designed the finite-state-machine workflow for calibration, monitoring, warning, GPS wait, and alert sent states
* Implemented accelerometer-based tilt, shock, and possible drop classification
* Added severity estimation and event history tracking
* Integrated GPS parsing from UART NMEA sentences
* Added buzzer warning behavior and SW2 cancel/reset logic
* Built dynamic AWS IoT JSON payloads containing event details, sensor data, network data, and GPS coordinates
* Improved the project narrative from a simple alert demo into a package-condition black box system

## Future Improvements

* Add persistent storage for event history
* Add battery-powered enclosure design for real shipping tests
* Add MQTT-based AWS IoT communication
* Add a mobile or web dashboard for event visualization
* Add sensor fusion with additional motion sensors
* Add machine learning-based event classification after collecting real package-handling data

## Author

Xiang Mao
B.S. Computer Engineering, University of California, Davis
GitHub: https://github.com/XiangM7
LinkedIn: https://www.linkedin.com/in/xiang-mao-78ab73301/
