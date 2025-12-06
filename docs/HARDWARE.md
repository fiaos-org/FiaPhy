# FiaPhy Hardware Setup Guide

## Overview

FiaPhy requires a **differential sensor architecture** - two identical sensors in thermally distinct enclosures. This guide covers physical construction, sensor selection, and enclosure design.

---

## Sensor Selection

### Recommended: Bosch BME280

**Why BME280?**
- Measures T/H/P in single package
- High accuracy (±0.01°C, ±3% RH, ±1 hPa)
- Low power (<1 mW)
- I2C interface (multiple addresses: 0x76, 0x77)
- Wide availability (~$5 USD)
- Validated against research paper specifications

**Alternatives:**
| Sensor | T | H | P | Notes |
|--------|---|---|---|-------|
| BME680 | ✓ | ✓ | ✓ | Adds gas sensor (not used by FiaPhy) |
| DHT22 + BMP280 | ✓ | ✓ | ✓ | Requires 2 sensors, slower response |
| SHT31 + BMP388 | ✓ | ✓ | ✓ | Higher accuracy, higher cost |

**Critical:** Both sensors must be **identical models and identical model types,** for accurate differential measurements.

---

## Enclosure Design

### Reference Node (Sensor 0)

**Purpose:** Measures true ambient air conditions without solar heating.

**Design Requirements:**
1. **Ventilation:** Free airflow around sensor
2. **Solar Shielding:** Blocks direct and diffuse sunlight
3. **Thermal Isolation:** Minimizes radiative heating
4. **Rain Protection:** Prevents water ingress

**Construction:**

```
┌─────────────────────────────────────┐
│     ┌───────────────────────┐       │  ← White/Reflective Outer Shell
│     │  ┌─────────────┐      │       │
│ AIR │  │   BME280    │  AIR │       │  ← Ventilation Slots
│ ═══►│  │  [Sensor]   │◄═══  │       │
│     │  └─────────────┘      │       │
│     └───────────────────────┘       │
│           ▲                          │
└───────────┼──────────────────────────┘
            │
         Mounting
```

**Materials:**
- PVC pipe (2-3 inch diameter)
- White spray paint (high solar reflectance)
- Mesh screen (insect protection)
- Mounting bracket

**Example Build:**
1. Cut 6-inch section of PVC pipe
2. Drill 8-10 ventilation holes (10mm diameter)
3. Cover holes with fine mesh
4. Mount sensor inside on standoff
5. Paint exterior white
6. Position vertically to prevent rain entry

---

### Flux Node (Sensor 1)

**Purpose:** Absorbs solar radiation and heats up measurably.

**Design Requirements:**
1. **Solar Absorption:** Maximizes energy capture
2. **Wind Protection:** Sealed against convective cooling
3. **Thermal Coupling:** Good contact with absorptive surface
4. **Minimal Thermal Mass:** Fast response time

**Construction:**

```
┌─────────────────────────────┐
│  Clear Dome (Glass/Acrylic) │  ← Transmits sunlight
├─────────────────────────────┤
│    ┌─────────────────┐      │
│    │    BME280       │      │  ← Sensor in sealed cavity
│    └─────────────────┘      │
├─────────────────────────────┤
│   Black Absorptive Base     │  ← Matte black paint (α ≈ 0.92)
└─────────────────────────────┘
```

**Materials:**
- Small project box (50mm × 50mm × 30mm)
- Clear acrylic dome or glass cover
- Matte black spray paint
- Thermal adhesive (optional)
- Silicone sealant

**Example Build:**
1. Paint interior of box matte black
2. Mount sensor on bottom surface
3. Seal box with clear acrylic top
4. Ensure airtight seal (use silicone)
5. Test: Box should heat up significantly in sunlight

**Coating Selection:**
| Coating | Absorptivity (α) | Notes |
|---------|------------------|-------|
| Matte black paint | 0.90-0.95 | Standard, easy to apply |
| Black anodizing | 0.88-0.92 | Durable, aluminum only |
| Carbon black | 0.95-0.98 | Professional grade |
| Glossy black | 0.85-0.88 | Lower performance |

---

## Physical Installation

### Placement Guidelines

**Both Enclosures:**
- Install **side-by-side** (< 50cm apart)
- Same altitude and orientation
- Clear view of sky (no obstructions >10° above horizon)
- Avoid heat sources (AC exhausts, chimneys)
- Stable mounting (prevents wind vibration)

**Height Recommendations:**
- Rooftop: 1-2 meters above surface
- Ground: 1.5-2.0 meters (standard meteorological height)
- Avoid mounting on hot surfaces (metal roofs)

**Orientation:**
- Flux node dome facing zenith (straight up)
- Reference node vertical for rain drainage

---

## Wiring Diagram

### Arduino Uno with Dual BME280

```
Arduino Uno          BME280 (0x76)        BME280 (0x77)
┌─────────┐          Reference Node       Flux Node
│         │
│  5V  ───┼──────────── VCC ──────────────── VCC
│  GND ───┼──────────── GND ──────────────── GND
│  A4  ───┼──────────── SDA ──────────────── SDA
│  A5  ───┼──────────── SCL ──────────────── SCL
│         │
└─────────┘
```

**I2C Address Configuration:**
- BME280 (0x76): SDO pin connected to GND
- BME280 (0x77): SDO pin connected to VCC

**Pull-up Resistors:** Most BME280 breakout boards include I2C pull-ups. If using bare chips, add 4.7kΩ resistors on SDA/SCL lines.

---

### ESP32 with Dual BME280

```
ESP32-DevKit       BME280 (0x76)        BME280 (0x77)
┌─────────┐
│         │
│  3.3V ──┼──────────── VCC ──────────────── VCC
│  GND  ──┼──────────── GND ──────────────── GND
│  GPIO21─┼──────────── SDA ──────────────── SDA
│  GPIO22─┼──────────── SCL ──────────────── SCL
│         │
└─────────┘
```

---

### Raspberry Pi with Dual BME280

```
Raspberry Pi 4     BME280 (0x76)        BME280 (0x77)
┌─────────┐
│  Pin 1  ├─ 3.3V ──── VCC ──────────────── VCC
│  Pin 6  ├─ GND  ──── GND ──────────────── GND
│  Pin 3  ├─ SDA  ──── SDA ──────────────── SDA
│  Pin 5  ├─ SCL  ──── SCL ──────────────── SCL
└─────────┘
```

**Enable I2C:**
```bash
sudo raspi-config
# Interface Options → I2C → Enable

# Verify devices
i2cdetect -y 1
```

---

## Verification Checklist

### Pre-Deployment Tests

**1. Sensor Detection**
```cpp
// Arduino
#include <Wire.h>
#include <Adafruit_BME280.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  Adafruit_BME280 bme_ref, bme_flux;
  
  if(bme_ref.begin(0x76)) Serial.println("✓ Reference sensor OK");
  else Serial.println("✗ Reference sensor FAIL");
  
  if(bme_flux.begin(0x77)) Serial.println("✓ Flux sensor OK");
  else Serial.println("✗ Flux sensor FAIL");
}
```

**2. Baseline Readings**

Indoors (room temperature, no sunlight):
- Both sensors should report **identical** temperatures (±0.2°C)
- Humidity within ±3%
- Pressure within ±1 hPa

**3. Differential Response Test**

Take sealed Flux node outdoors in sunlight:
- Flux temperature should rise **2-5°C above** Reference within 60 seconds
- Larger differential indicates good solar absorption
- If no differential: Check sealing and black coating

**4. Ventilation Test**

Blow on Reference node:
- Temperature should change within 5 seconds
- If slow: Improve ventilation

---

## Troubleshooting

### Problem: No Temperature Differential

**Causes:**
- ✗ Flux node not sealed (air leaks)
- ✗ Poor solar absorption (use matte black, not glossy)
- ✗ Dome blocking UV (use glass, not some plastics)
- ✗ Testing indoors or under clouds

**Solution:** Test on clear sunny day at solar noon (highest irradiance)

---

### Problem: Reference Node Heats Up

**Causes:**
- ✗ Insufficient ventilation
- ✗ Dark-colored enclosure
- ✗ Mounting on hot surface

**Solution:** Increase ventilation holes, paint white, elevate on standoff

---

### Problem: Erratic Readings

**Causes:**
- ✗ I2C wiring issues (long cables, no pull-ups)
- ✗ Power supply noise
- ✗ Vibration (wind shaking mount)

**Solution:** 
- Shorten I2C cables (<30cm ideal)
- Add 100nF capacitors near sensor VCC pins
- Secure mounting

---

## Advanced Configurations

### Multi-Sensor Arrays (3+ pairs)

For spatial averaging or redundancy:

```
Reference Nodes        Flux Nodes
    (0x76)               (0x77)
       ║                    ║
    (0x76)               (0x77)
       ║                    ║
    (0x76)               (0x77)
       ╚════════════════════╝
          I2C Multiplexer
          (TCA9548A)
```

FiaPhy supports up to 8 sensor pairs via I2C multiplexing.

---

### Calibration Test Rig

For research-grade validation:

1. Install reference pyranometer alongside FiaPhy
2. Log both systems simultaneously
3. Compare readings over 24-hour period
4. Adjust `CalibrationParams` to minimize RMS error

---

## Safety & Maintenance

**Electrical:**
- Use appropriate voltage (3.3V for BME280, NOT 5V)
- Weatherproof all outdoor connections
- Consider lightning protection for permanent installations

**Mechanical:**
- Check mounting security monthly
- Clean enclosures quarterly (dust reduces accuracy)
- Inspect seals after heavy rain

**Sensor Lifespan:**
- BME280 rated for 5+ years outdoor use
- Replace if readings drift >5% from baseline

---

## Bill of Materials

**Minimal Setup (~$30 USD):**
- 2× Adafruit BME280 breakout boards ($10 each)
- Arduino Uno clone ($8)
- 1× PVC pipe enclosure ($2)
- 1× Small project box ($2)
- Spray paint (white + black) ($5)
- Wire, screws, mounting hardware ($3)

**Professional Setup (~$120 USD):**
- 2× BME280 sensors
- ESP32 with WiFi ($12)
- Custom 3D-printed enclosures ($20)
- Weather-resistant connectors ($15)
- Solar panel + battery (optional, $50)
- Datalogger SD card module ($5)

---

## Next Steps

1. Build enclosures
2. Wire sensors
3. Run verification tests
4. Deploy hardware
5. Install FiaPhy firmware
6. Calibrate system

See [Getting Started Guide](GETTING_STARTED.md) for software installation.
