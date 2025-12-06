Copyrights (c) www.FiaOS.org and www.nekshadesilva.com. All Rights Reserved.

<img src="http://fiaos.org/assets/media/fiaphy-clean.png" >

Repository restored after previous FiaOS core repository commit clearance.

Differential Temporal Derivative Soft-Sensing (DTDSS) library for reconstructing solar radiation and heat flux from commodity environmental sensors.

**Version:** 1.0.1  
**Research Paper:** "Temporal Derivative Soft-Sensing and Reconstructing Solar Radiation and Heat Flux" by Neksha V. DeSilva, FiaOS.org

## Platform Support

<p align="center">
  <img src="https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=Arduino&logoColor=white" alt="Arduino"/>
  <img src="https://img.shields.io/badge/ESP32-000000?style=for-the-badge&logo=Espressif&logoColor=white" alt="ESP32"/>
  <img src="https://img.shields.io/badge/Raspberry%20Pi-A22846?style=for-the-badge&logo=Raspberry%20Pi&logoColor=white" alt="Raspberry Pi"/>
  <img src="https://img.shields.io/badge/PlatformIO-FF7F00?style=for-the-badge&logo=data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMjUwMCIgaGVpZ2h0PSIyNTAwIiB2aWV3Qm94PSIwIDAgMjU2IDI1NiIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiBwcmVzZXJ2ZUFzcGVjdFJhdGlvPSJ4TWlkWU1pZCI+PHBhdGggZD0iTTEyOCAwQzU3LjMgMCAwIDU3LjMgMCAxMjhzNTcuMyAxMjggMTI4IDEyOCAxMjgtNTcuMyAxMjgtMTI4UzE5OC43IDAgMTI4IDB6bTAgMjM4LjljLTYxLjQgMC0xMTAuOS00OS41LTExMC45LTExMC45UzY2LjYgMTcuMSAxMjggMTcuMXMxMTAuOSA0OS41IDExMC45IDExMC45LTQ5LjUgMTEwLjktMTEwLjkgMTEwLjl6IiBmaWxsPSIjRkZGIi8+PHBhdGggZD0iTTE2NC41IDEwOC4xTDEyOCA4Ni41bC0zNi41IDIxLjZ2NDMuMmwzNi41IDIxLjYgMzYuNS0yMS42eiIgZmlsbD0iI0ZGRiIvPjwvc3ZnPg==" alt="PlatformIO"/>
</p>

- Arduino (AVR, ARM Cortex-M0/M3/M4)
- ESP32/ESP8266  
- Raspberry Pi (Linux ARM)
- PlatformIO

## Installation

### Arduino IDE

**Requirements:** Arduino IDE 1.8.13+ or Arduino IDE 2.x  
**Download:** https://www.arduino.cc/en/software

```
1. Open Arduino IDE
2. Sketch → Include Library → Manage Libraries
3. Search "FiaPhy"
4. Click Install
```

**Manual Installation:**
```
1. Download FiaPhy-1.0.1.zip from Releases
2. Sketch → Include Library → Add .ZIP Library
3. Select downloaded file
```

### PlatformIO

**Requirements:** PlatformIO Core 6.0+ or VS Code + PlatformIO extension  
**Download:** https://platformio.org/install

**platformio.ini:**
```ini
[env:yourboard]
platform = atmelavr  ; or espressif32, raspberrypi, etc.
board = uno          ; or esp32dev, rpipico, etc.
lib_deps = 
    fiaos-org/FiaPhy@^1.0.1
    adafruit/Adafruit BME280 Library@^2.2.2
```

### Raspberry Pi

**Requirements:** Raspberry Pi OS, GCC 8.0+, CMake 3.10+

```bash
sudo apt update
sudo apt install git cmake g++ libi2c-dev
git clone https://github.com/fiaos-org/FiaPhy.git
cd FiaPhy/examples/RaspberryPi
chmod +x build.sh
./build.sh
./raspberry_pi_example
```

## Quick Start

### Minimal Example

```cpp
#include <FiaPhy.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 refSensor;   // Address 0x76
Adafruit_BME280 fluxSensor;  // Address 0x77
FiaPhy::DTDSS system;

void setup() {
    Serial.begin(115200);
    
    refSensor.begin(0x76);
    fluxSensor.begin(0x77);
    
    system.configure(6.9271, 79.8612, 100.0);  // Latitude, Longitude, Altitude(m)
}

void loop() {
    system.feedReferenceData(refSensor.readTemperature(), 
                            refSensor.readHumidity(), 
                            refSensor.readPressure() / 100.0);
    
    system.feedFluxData(fluxSensor.readTemperature(), 
                       fluxSensor.readHumidity(), 
                       fluxSensor.readPressure() / 100.0);
    
    if (system.isFrameReady()) {
        FiaPhy::RadiationResult result = system.compute();
        if (result.valid) {
            Serial.print("GHI: ");
            Serial.print(result.ghi_Wm2);
            Serial.println(" W/m²");
        }
    }
    
    delay(3000);
}
```

## Hardware Setup

**Reference Sensor:** Ventilated, reflective white housing  
**Flux Sensor:** Sealed black-body enclosure with transparent dome  
**Wiring:** I2C (SDA/SCL) to both sensors, different addresses (0x76, 0x77)


## Research

Based on research by Neksha DeSilva  
FiaOS.org (research@fiaos.org)
