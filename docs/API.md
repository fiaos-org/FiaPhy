# FiaPhy API Reference

## Core Classes

### `FiaPhy::SensorHub`

**Purpose:** Synchronizes asynchronous sensor readings and validates data integrity.

#### Public Methods

```cpp
bool begin(uint8_t sensor_count)
```
Initialize hub for specified number of sensor pairs (2-8).
- **Parameters:** `sensor_count` - Number of complete sensor units
- **Returns:** `true` on success
- **Example:**
  ```cpp
  FiaPhy::SensorHub hub;
  if(!hub.begin(2)) {
      Serial.println("Initialization failed");
  }
  ```

---

```cpp
ValidationResult feedTemperature(float temp_C, uint8_t sensor_id)
ValidationResult feedHumidity(float humidity_RH, uint8_t sensor_id)
ValidationResult feedPressure(float pressure_hPa, uint8_t sensor_id)
```
Feed individual sensor readings. Order doesn't matter - library synchronizes internally.
- **Parameters:** 
  - Value in appropriate units (°C, %, hPa)
  - `sensor_id` - Sensor identifier (0 to sensor_count-1)
- **Returns:** `ValidationResult` with error details if validation fails
- **Example:**
  ```cpp
  auto result = hub.feedTemperature(25.3, 0);
  if(!result.isOk()) {
      Serial.println(result.message);
  }
  ```

**Validation Checks:**
- Range validation (T: -50 to 85°C, H: 0-100%, P: 300-1200 hPa)
- Jump detection (prevents spurious readings)
- Sensor ID bounds checking

---

```cpp
bool isFrameReady() const
```
Check if complete synchronized frame is available.
- **Returns:** `true` when all sensors have provided T/H/P triplets
- **Example:**
  ```cpp
  if(hub.isFrameReady()) {
      // Process data
  }
  ```

---

```cpp
const SensorFrame* getFrames() const
```
Retrieve synchronized sensor frames.
- **Returns:** Pointer to array of `SensorFrame` structures
- **Note:** Only valid when `isFrameReady()` returns `true`

---

```cpp
void acknowledgeFrame()
```
Mark current frame as consumed. Call after processing to ready system for next frame.

---

### `FiaPhy::PhysicsEngine`

**Purpose:** Implements DTDSS algorithm and computes solar radiation.

#### Public Methods

```cpp
void setCalibration(const CalibrationParams& params)
```
Configure calibration parameters.
- **Parameters:** `CalibrationParams` structure
- **Example:**
  ```cpp
  FiaPhy::CalibrationParams calib;
  calib.thermal_time_constant_s = 30.0f;
  calib.solar_absorptivity = 0.90f;
  engine.setCalibration(calib);
  ```

**Calibration Parameters:**
| Parameter | Default | Units | Description |
|-----------|---------|-------|-------------|
| `thermal_time_constant_s` | 30.0 | seconds | Sensor thermal inertia (τ) |
| `solar_absorptivity` | 0.90 | - | Black body coating (α) |
| `convective_area_m2` | 6.25e-6 | m² | Sensor surface area |
| `heat_capacity_J_K` | 0.5 | J/K | Thermal mass (m·Cp) |
| `self_heating_offset_C` | 0.8 | °C | Electrical self-heating |

---

```cpp
SolarFlux compute(const SensorFrame* frames, uint8_t sensor_count)
```
Compute solar radiation from synchronized frames.
- **Parameters:** 
  - `frames` - Array from `SensorHub::getFrames()`
  - `sensor_count` - Number of sensors
- **Returns:** `SolarFlux` structure with results

**SolarFlux Structure:**
```cpp
struct SolarFlux {
    float irradiance_Wm2;       // Global Horizontal Irradiance
    float heat_flux_Wm2;        // Convective heat flux
    float sol_air_excess_C;     // Temperature excess (T_sol)
    float cloud_fraction;       // Estimated cloud cover (0-1)
    float confidence;           // Result confidence (0-1)
};
```

---

### `FiaPhy::INRFilter`

**Purpose:** Inertial Noise Reduction for derivative estimation.

#### Public Methods

```cpp
float update(float raw_value)
```
Process new value through adaptive filter.
- **Returns:** Filtered value

---

```cpp
float computeProjection(float thermal_time_constant_s)
```
Compute inertial projection: T_proj = T_filt + τ·(dT/dt)
- **Returns:** Projected temperature

---

## Data Types

### `SensorReading`
Raw sensor input structure.

```cpp
struct SensorReading {
    float temperature_C;
    float humidity_RH;
    float pressure_hPa;
    uint8_t sensor_id;
    uint32_t timestamp_ms;
    
    bool isValid() const;
};
```

---

### `SensorFrame`
Complete synchronized triplet.

```cpp
struct SensorFrame {
    float temperature_C;
    float humidity_RH;
    float pressure_hPa;
    uint8_t sensor_id;
    bool complete;
};
```

---

### `AtmosphericState`
Derived thermodynamic properties.

```cpp
struct AtmosphericState {
    float air_density_kg_m3;
    float vapor_pressure_hPa;
    float saturation_pressure_hPa;
    float specific_enthalpy_kJ_kg;
    float mixing_ratio;
    float temperature_K;
};
```

---

### `ValidationResult`
Error reporting structure.

```cpp
struct ValidationResult {
    ErrorCode code;
    const char* message;
    uint8_t faulty_sensor_id;
    
    bool isOk() const;
};
```

**Error Codes:**
- `OK` - No error
- `SENSOR_ASYMMETRY` - Unequal sensor counts
- `INCOMPLETE_TRIPLET` - Missing T/H/P value
- `VALUE_OUT_OF_RANGE` - Physically impossible reading
- `UNREALISTIC_JUMP` - Value changed too rapidly
- `INSUFFICIENT_SENSORS` - Less than 2 sensor pairs
- `BUFFER_OVERFLOW` - More than 8 sensor pairs

---

## Physics Functions

### `Thermodynamics` Namespace

```cpp
float calculateSaturationPressure(float temp_C)
```
Magnus formula for saturation vapor pressure.

---

```cpp
float calculateVaporPressure(float temp_C, float humidity_RH)
```
Actual vapor pressure from RH.

---

```cpp
float calculateAirDensity(float pressure_hPa, float temp_C, float humidity_RH)
```
**Altitude-independent** moist air density calculation.

---

```cpp
AtmosphericState computeAtmosphericState(const SensorFrame& frame)
```
Complete thermodynamic state derivation.

---

## Platform Support

### Logger

Cross-platform logging abstraction.

```cpp
FiaPhy::Logger::initialize(LogLevel min_level);
FiaPhy::Logger::info("Temperature: %.2f", temp);
FiaPhy::Logger::warn("Sensor deviation detected");
FiaPhy::Logger::error("Initialization failed");
FiaPhy::Logger::debug("Frame #%d ready", frame_count);
```

**Log Levels:** `DEBUG`, `INFO`, `WARN`, `ERROR`, `NONE`

**Platform Routing:**
- Arduino → `Serial.print()`
- ESP32 → `esp_log`
- Linux/POSIX → `printf()`

---

## Constants

Defined in `FiaPhy::Constants` namespace:

```cpp
R_DRY_AIR = 287.058f        // J/(kg·K)
R_WATER_VAPOR = 461.495f    // J/(kg·K)
EPSILON = 0.622f            // Rd/Rv ratio
ZERO_CELSIUS = 273.15f      // Kelvin offset
MAGNUS_A = 17.67f           // Magnus coefficient
MAGNUS_B = 243.5f           // Magnus coefficient (°C)
MAGNUS_C = 6.112f           // Magnus coefficient (hPa)
```

---

## Usage Pattern

**Standard workflow:**

```cpp
#include <FiaPhy.h>

FiaPhy::SensorHub hub;
FiaPhy::PhysicsEngine engine;

void setup() {
    FiaPhy::initialize();
    hub.begin(2);
}

void loop() {
    // 1. Read sensors
    float temp = sensor.readTemperature();
    float humid = sensor.readHumidity();
    float press = sensor.readPressure();
    
    // 2. Feed to hub (order doesn't matter)
    hub.feedTemperature(temp, sensor_id);
    hub.feedHumidity(humid, sensor_id);
    hub.feedPressure(press, sensor_id);
    
    // 3. Process when ready
    if(hub.isFrameReady()) {
        FiaPhy::SolarFlux result = engine.compute(
            hub.getFrames(), 
            hub.getSensorCount()
        );
        
        // 4. Use results
        Serial.print("GHI: ");
        Serial.println(result.irradiance_Wm2);
        
        // 5. Acknowledge
        hub.acknowledgeFrame();
    }
}
```

---

## Advanced Features

### Custom Calibration

Auto-calibrate thermal time constant:

```cpp
// Wait for sunset/shading event
// Record temperature decay
float tau = measureDecayConstant();

FiaPhy::CalibrationParams calib;
calib.thermal_time_constant_s = tau;
engine.setCalibration(calib);
```

### Multi-Sensor Arrays

Support up to 8 sensor pairs:

```cpp
hub.begin(4);  // 4 reference + 4 flux nodes

// Average multiple flux readings for noise reduction
```

### Error Handling

```cpp
auto result = hub.feedTemperature(temp, id);
if(!result.isOk()) {
    switch(result.code) {
        case ErrorCode::VALUE_OUT_OF_RANGE:
            // Handle range error
            break;
        case ErrorCode::UNREALISTIC_JUMP:
            // Handle jump error
            break;
    }
}
```

---

## Performance

**Memory Footprint:**
- Core state: ~60 bytes RAM
- Per sensor: ~18 bytes
- Stack usage: <200 bytes

**Computational Complexity:**
- Thermodynamics: O(1)
- INR filter: O(1)
- Total per frame: <1ms on 8-bit AVR

**Accuracy:**
- Steady-state: ±10% after calibration
- Transient response: 5-10 seconds
- Altitude range: 0-5000m (automatic)
