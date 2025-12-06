# FiaPhy Getting Started Guide

Get your solar radiation monitoring system running in under 30 minutes.

---

## Prerequisites

- ✅ Hardware: 2× BME280 sensors + Arduino/ESP32/Raspberry Pi
- ✅ USB cable for programming
- ✅ Internet connection (for library installation)

---

## Installation

### Arduino IDE

**Step 1:** Download Library
1. Download latest release from [GitHub](https://github.com/fiaos-org/FiaPhy/releases)
2. Extract ZIP file

**Step 2:** Install Library
1. Open Arduino IDE
2. Sketch → Include Library → Add .ZIP Library
3. Select downloaded `FiaPhy-1.0.0.zip`

**Step 3:** Install Dependencies
```
Tools → Manage Libraries
Search: "Adafruit BME280"
Install: Adafruit BME280 Library (v2.2.2+)
```

**Step 4:** Load Example
```
File → Examples → FiaPhy → Arduino_BME280
```

**Step 5:** Configure Hardware
Edit these lines if your I2C addresses differ:
```cpp
Adafruit_BME280 bme_ref;  // Change address if needed: bme_ref.begin(0x76)
Adafruit_BME280 bme_flux; // Change address if needed: bme_flux.begin(0x77)
```

**Step 6:** Upload
1. Connect Arduino via USB
2. Select board: Tools → Board → Arduino Uno
3. Select port: Tools → Port → (your Arduino)
4. Click Upload ⬆️

**Step 7:** View Results
```
Tools → Serial Monitor
Set baud rate: 115200
```

You should see:
```
=== FiaPhy Solar Radiation Monitor ===
Initialization complete!
Waiting for sensor data...

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Frame #1
Reference Node: T: 23.45°C  H: 62.3%  P: 1013.25 hPa
Flux Node:      T: 25.78°C  H: 60.1%  P: 1013.20 hPa
═══ SOLAR RADIATION ═══
  GHI: 487.3 W/m²
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

### PlatformIO

**Step 1:** Create Project
```bash
platformio init --board esp32dev
```

**Step 2:** Edit platformio.ini
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
    fiaos-org/FiaPhy @ ^1.0.0
    adafruit/Adafruit BME280 Library @ ^2.2.2
```

**Step 3:** Create main.cpp
```cpp
#include <Arduino.h>
#include <FiaPhy.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme_ref, bme_flux;
FiaPhy::SensorHub hub;
FiaPhy::PhysicsEngine engine;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  bme_ref.begin(0x76);
  bme_flux.begin(0x77);
  
  FiaPhy::initialize();
  hub.begin(2);
}

void loop() {
  static uint32_t last_time = 0;
  if(millis() - last_time >= 1000) {
    last_time = millis();
    
    // Read sensors
    bme_ref.takeForcedMeasurement();
    bme_flux.takeForcedMeasurement();
    
    // Feed data
    hub.feedTemperature(bme_ref.readTemperature(), 0);
    hub.feedHumidity(bme_ref.readHumidity(), 0);
    hub.feedPressure(bme_ref.readPressure() / 100.0f, 0);
    
    hub.feedTemperature(bme_flux.readTemperature(), 1);
    hub.feedHumidity(bme_flux.readHumidity(), 1);
    hub.feedPressure(bme_flux.readPressure() / 100.0f, 1);
    
    // Process
    if(hub.isFrameReady()) {
      FiaPhy::SolarFlux result = engine.compute(
        hub.getFrames(), 
        hub.getSensorCount()
      );
      
      Serial.printf("GHI: %.1f W/m²\n", result.irradiance_Wm2);
      hub.acknowledgeFrame();
    }
  }
}
```

**Step 4:** Build & Upload
```bash
platformio run --target upload
platformio device monitor
```

---

### Raspberry Pi

**Step 1:** Install Dependencies
```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git
sudo apt-get install -y i2c-tools libi2c-dev
```

**Step 2:** Enable I2C
```bash
sudo raspi-config
# Interface Options → I2C → Yes
sudo reboot
```

**Step 3:** Verify Sensors
```bash
i2cdetect -y 1
```
Should show devices at 0x76 and 0x77:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
...
70: -- -- -- -- -- -- 76 77  
```

**Step 4:** Clone FiaPhy
```bash
cd ~
git clone https://github.com/fiaos-org/FiaPhy.git
cd FiaPhy
```

**Step 5:** Build Example
```bash
cd examples/RaspberryPi
chmod +x build.sh
./build.sh
```

**Step 6:** Run
```bash
sudo ./solar_monitor
```

Output logs to `solar_data.csv` for analysis.

---

## Quick Verification

### Indoor Test (No Sunlight)

Expected behavior:
- Both sensors report similar temperatures (within 1°C)
- GHI near 0 W/m² (indoor lighting is negligible)
- No errors in serial output

If Reference and Flux temps differ >2°C indoors → Check enclosures

---

### Outdoor Test (Sunlight)

Place system in direct sunlight for 2 minutes.

Expected behavior:
- Flux temperature rises 3-8°C above Reference
- GHI increases to 400-1000 W/m² (depending on sun angle)
- GHI drops to ~100 W/m² when shaded

**Benchmark Values:**
| Condition | Expected GHI |
|-----------|--------------|
| Clear noon (summer) | 900-1100 W/m² |
| Clear noon (winter) | 600-800 W/m² |
| Partly cloudy | 300-600 W/m² |
| Overcast | 50-200 W/m² |
| Indoors | 0-10 W/m² |

---

## Calibration

### Basic Calibration (Recommended)

Measure thermal time constant (τ) for your enclosure:

1. Place system in sunlight for 5 minutes
2. Quickly shade Flux node (cover with hand)
3. Watch temperature decay in serial monitor
4. Measure time to drop 63% of difference (this is τ)

Example:
```
Initial: T_flux = 30°C, T_ref = 25°C → Difference = 5°C
After shading:
  10s: 28.2°C → 3.2°C difference
  20s: 26.8°C → 1.8°C difference  ← This is 63% decay (τ ≈ 20s)
  30s: 26.1°C → 1.1°C difference
```

Update code:
```cpp
FiaPhy::CalibrationParams calib;
calib.thermal_time_constant_s = 20.0f;  // Your measured value
engine.setCalibration(calib);
```

---

### Advanced Calibration (Optional)

If you have a reference pyranometer:

```cpp
// Collect paired data
for(int i = 0; i < 100; i++) {
  float GHI_reference = pyranometer.read();
  float GHI_fiaphy = result.irradiance_Wm2;
  
  // Calculate scaling factor
  total_error += (GHI_fiaphy - GHI_reference);
}

// Adjust calibration
calib.solar_absorptivity *= (average_reference / average_fiaphy);
```

---

## Common Issues

### "Sensor not found"

**Symptom:** Serial output shows sensor initialization failure

**Solutions:**
1. Check wiring (SDA/SCL swapped?)
2. Verify I2C addresses:
   ```cpp
   Wire.begin();
   Wire.beginTransmission(0x76);
   if(Wire.endTransmission() == 0) Serial.println("Found 0x76");
   ```
3. Check voltage (BME280 is 3.3V, NOT 5V tolerant on some breakouts)

---

### GHI Always Zero

**Symptom:** Readings stay at 0 W/m² even in sunlight

**Solutions:**
1. Verify Flux sensor is in sealed BLACK enclosure (not ventilated)
2. Test differential:
   ```cpp
   Serial.print("Temp diff: ");
   Serial.println(T_flux - T_ref);
   ```
   Should be >2°C in sunlight
3. Check `solar_absorptivity` parameter (default 0.90)

---

### Unrealistic Values (>1400 W/m²)

**Symptom:** GHI shows impossibly high values

**Solutions:**
1. Check for additional heat sources (sensor near hot surface?)
2. Verify self-heating offset:
   ```cpp
   calib.self_heating_offset_C = 1.2f;  // Increase if needed
   ```
3. Ensure Reference sensor is properly ventilated

---

### Readings Noisy/Jumping

**Symptom:** GHI fluctuates wildly second-to-second

**Solutions:**
1. INR filter needs more time to stabilize (wait 30 seconds after startup)
2. Check for vibration (wind shaking mount?)
3. Enable debug logging:
   ```cpp
   FiaPhy::Logger::setLevel(FiaPhy::LogLevel::DEBUG);
   ```

---

## Next Steps

### Data Logging

Save readings to SD card:
```cpp
#include <SD.h>
File dataFile = SD.open("solar.csv", FILE_WRITE);
dataFile.printf("%lu,%.2f,%.2f\n", millis(), result.irradiance_Wm2, temp);
dataFile.close();
```

---

### Web Dashboard

Send data via WiFi:
```cpp
#include <WiFi.h>
#include <HTTPClient.h>

HTTPClient http;
http.begin("http://your-server.com/api/solar");
http.addHeader("Content-Type", "application/json");

String json = "{\"ghi\":" + String(result.irradiance_Wm2) + "}";
http.POST(json);
```

---

### Integration with Home Assistant

MQTT publishing:
```cpp
#include <PubSubClient.h>

client.publish("home/solar/ghi", String(result.irradiance_Wm2).c_str());
```

---

## Getting Help

**Documentation:**
- [API Reference](API.md)
- [Hardware Guide](HARDWARE.md)
- [Research Paper](PAPER.pdf)

**Community:**
- GitHub Issues: https://github.com/fiaos-org/FiaPhy/issues
- Email: research@fiaos.org
- Website: https://www.fiaos.org

**Debugging:**
Enable verbose logging:
```cpp
FiaPhy::Logger::setLevel(FiaPhy::LogLevel::DEBUG);
```

---

## Success Checklist

- Sensors detected and reading valid data
- Temperature differential observed in sunlight (Flux > Reference)
- GHI values in expected range (0-1100 W/m²)
- System runs stable for 10+ minutes without errors
- Results correlate with weather conditions (clear vs cloudy)

**You're ready to deploy! 🎉**

Consider sharing your project:
- Post photos of your setup
- Contribute calibration data
- Report accuracy comparisons
- Submit improvements via Pull Requests

---

## Example Projects

**Agriculture:** Evapotranspiration monitoring for irrigation scheduling

**Solar Energy:** Real-time PV output prediction

**Weather Stations:** Add radiation to existing sensor arrays

**Research:** Climate data collection in remote locations

**Education:** Physics demonstration of thermodynamics principles

---

## License

MIT License - Free for personal, educational, and commercial use.

See [LICENSE](../LICENSE) for details.
