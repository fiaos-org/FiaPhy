# Contributing to FiaPhy

Guidelines for contributing to the project.

---

## Code of Conduct

All contributors must:
- Be respectful and inclusive
- Focus on constructive feedback
- Prioritize scientific accuracy and engineering rigor
- Document contributions thoroughly

---

## How to Contribute

### Reporting Bugs

Before submitting:
1. Check existing [Issues](https://github.com/fiaos-org/FiaPhy/issues)
2. Verify you're using the latest version
3. Test with minimal example code

Bug Report Template:
```markdown
**Description:** Clear description of the issue

**Hardware:**
- Platform: Arduino Uno / ESP32 / Raspberry Pi
- Sensors: BME280 at addresses 0x76, 0x77
- Enclosures: [describe Reference and Flux node designs]

**Code:**
```cpp
// Minimal code that reproduces the issue
```

**Expected Behavior:** What should happen

**Actual Behavior:** What actually happens

**Serial Output:**
```
// Paste relevant logs
```

**Additional Context:** Environmental conditions, calibration params, etc.
```

---

### Suggesting Enhancements

Feature Request Template:
```markdown
**Use Case:** Why is this feature needed?

**Proposed Solution:** How should it work?

**Alternatives Considered:** What other approaches did you evaluate?

**Implementation Notes:** Technical considerations
```

---

### Pull Requests

#### Process

1. Fork the repository
    ```bash
    git clone https://github.com/YOUR_USERNAME/FiaPhy.git
    cd FiaPhy
    git checkout -b feature/your-feature-name
    ```

2. Make changes
    - Follow code style guidelines
    - Add tests if applicable
    - Update documentation

3. Test thoroughly
    - Compile for Arduino, ESP32, and Raspberry Pi
    - Verify examples still work
    - Check for memory leaks on embedded platforms

4. Commit with clear messages
    ```bash
    git commit -m "Add INR filter alpha tuning parameter
    
    - Expose alpha_min and alpha_max in CalibrationParams
    - Add documentation for manual tuning
    - Update Arduino example with custom alpha values"
    ```

5. Push and create PR
    ```bash
    git push origin feature/your-feature-name
    ```

---

## Code Style Guidelines

### C++ Standards

Use C++11 for maximum compatibility. Avoid C++14/17/20 features.

```cpp
// Valid
constexpr float CONSTANT = 1.23f;
auto value = calculateSomething();

// Invalid
std::optional<float> value;  // C++17
inline constexpr float CONSTANT = 1.23f;  // C++17
```

---

### Naming Conventions

```cpp
// Classes: PascalCase
class PhysicsEngine { };

// Functions/Methods: camelCase
float calculateAirDensity();

// Variables: snake_case
float air_density_kg_m3;

// Constants: UPPER_SNAKE_CASE
const float R_DRY_AIR = 287.058f;

// Private members: trailing underscore
class MyClass {
private:
     float value_;
};
```

---

### Header Guards

```cpp
#ifndef FIAPHY_MYHEADER_H
#define FIAPHY_MYHEADER_H

// ... content ...

#endif // FIAPHY_MYHEADER_H
```

---

### Documentation

Use Doxygen-style comments for public APIs.

```cpp
/**
 * @brief Calculate moist air density from measured state variables
 * 
 * Implements altitude-independent density calculation using measured
 * atmospheric pressure instead of altitude lookup tables.
 * 
 * @param pressure_hPa Atmospheric pressure in hectopascals
 * @param temp_C Temperature in degrees Celsius
 * @param humidity_RH Relative humidity (0-100%)
 * @return Air density in kg/m³
 * 
 * @note Uses measured pressure for altitude adaptation
 * @see Section 2.2 of research paper for derivation
 */
inline float calculateAirDensity(float pressure_hPa, float temp_C, float humidity_RH);
```

---

### Memory Management

For embedded systems:

```cpp
// Static allocation
constexpr uint8_t BUFFER_SIZE = 10;
float buffer[BUFFER_SIZE];

// Stack allocation
SensorFrame frame;

// Avoid dynamic allocation (heap fragmentation on MCU)
float* buffer = new float[size];  // Do not use
```

---

### Platform Independence

```cpp
#if defined(ARDUINO)
     Serial.println("Arduino");
#elif defined(__linux__)
     printf("Linux\n");
#else
     // Generic fallback
#endif

// Preferred: Use Logger class
FiaPhy::Logger::info("Platform-independent message");
```

---

## Testing Contributions

### Manual Testing Checklist

- [ ] Compiles without warnings on Arduino Uno
- [ ] Compiles without warnings on ESP32
- [ ] Compiles without warnings on Linux (g++)
- [ ] Examples run successfully
- [ ] Memory usage within limits (< 60 bytes RAM per sensor)
- [ ] No stack overflow on 8-bit AVR
- [ ] Validates against research paper equations

---

### Hardware Testing

If you have hardware setup:
1. Test with real BME280 sensors
2. Verify differential temperature response
3. Compare GHI output to weather reports
4. Document calibration parameters used

---

## Documentation Contributions

### Updating API Documentation

When adding new public methods:
1. Update `docs/API.md`
2. Add example code
3. Document parameters and return values
4. Reference research paper section if applicable

---

### Improving Examples

Examples should be:
- Minimal dependencies
- Well-commented
- Error-handling code included
- Feature-specific

---

### Translations

Translation structure:
```
docs/
├── en/  (English - primary)
├── es/  (Spanish)
├── zh/  (Chinese)
└── ...
```

---

## Areas Needing Contribution

### High Priority

- [ ] Auto-calibration algorithm (Section 7.2 of paper)
- [ ] Solar geometry calculations (clear-sky GHI from lat/lon/time)
- [ ] Infrared compensation (Section 10.4 of paper)
- [ ] Wind speed integration (Section 6 of paper)
- [ ] Unit test framework

---

### Medium Priority

- [ ] Python bindings (ctypes wrapper)
- [ ] MQTT publishing example
- [ ] Home Assistant integration
- [ ] 3D-printable enclosure designs (OpenSCAD)
- [ ] PCB design for integrated sensor board

---

### Low Priority

- [ ] Web dashboard (React/Vue)
- [ ] Mobile app (React Native)
- [ ] Cloud data aggregation service
- [ ] Machine learning augmentation (Section 10.2)

---

## Scientific Rigor

FiaPhy is based on peer-reviewed research. Contributions must maintain scientific accuracy.

When proposing algorithm changes:
1. Provide physical justification
2. Reference academic sources
3. Show validation data
4. Discuss error bounds
5. Consider edge cases (altitude, temperature extremes)

Do not:
- Add magic numbers without derivation
- Introduce ML models without physics grounding
- Sacrifice accuracy for convenience
- Ignore dimensional analysis

---

## Licensing

By contributing, you agree that your contributions will be licensed under the MIT License.

You certify that:
- The contribution is your original work OR
- You have rights to submit it under MIT License
- You understand the contribution becomes part of the public codebase

---

## Recognition

Contributors will be acknowledged in:
- `CONTRIBUTORS.md` file
- Release notes
- Research paper acknowledgments (for significant contributions)

---

## Questions?

- Technical questions: Open a [Discussion](https://github.com/fiaos-org/FiaPhy/discussions)
- Private inquiries: research@fiaos.org
- Research paper: [link to published paper]

