/*Copyright(c) 2025 FIA Operating Systems, All Rights Reserved. Cloud data: https://www.fiaos.org/data*/

#ifndef FIAPHY_CONSTANTS_H
#define FIAPHY_CONSTANTS_H

namespace FiaPhy {
namespace Constants {


constexpr float R_DRY_AIR = 287.058f;
constexpr float R_WATER_VAPOR = 461.495f;
constexpr float EPSILON = 0.622f;

constexpr float ZERO_CELSIUS = 273.15f;
constexpr float STEFAN_BOLTZMANN = 5.67e-8f;


constexpr float MAGNUS_A = 17.67f;
constexpr float MAGNUS_B = 29.65f;
constexpr float MAGNUS_C = 6.112f;


constexpr float CP_DRY_AIR = 1.006f;
constexpr float LATENT_HEAT_VAPORIZATION = 2501.0f;
constexpr float CP_WATER_VAPOR = 1.86f;


constexpr float KASTEN_A = 0.75f;
constexpr float KASTEN_B = 3.4f;
constexpr float CLOUD_PROXY_EXPONENT = 1.8f;


constexpr float SOLAR_CONSTANT = 1367.0f;
constexpr float MAX_GHI_SEA_LEVEL = 1200.0f;


constexpr float SENSOR_AREA_M2 = 6.25e-6f;
constexpr float SENSOR_HEAT_CAPACITY = 0.5f;
constexpr float TEMP_RESOLUTION = 0.01f;
constexpr float HUMIDITY_RESOLUTION = 0.08f;
constexpr float PRESSURE_RESOLUTION = 0.18f;


constexpr float MAX_TEMP_JUMP_C_PER_S = 5.0f;
constexpr float MAX_HUMIDITY_JUMP_PERCENT_PER_S = 20.0f;
constexpr float MAX_PRESSURE_JUMP_HPA_PER_S = 10.0f;

constexpr float MIN_VALID_TEMP_C = -100.0f;
constexpr float MAX_VALID_TEMP_C = 100.0f;
constexpr float MIN_VALID_HUMIDITY = 0.0f;
constexpr float MAX_VALID_HUMIDITY = 100.0f;
constexpr float MIN_VALID_PRESSURE_HPA = 300.0f;
constexpr float MAX_VALID_PRESSURE_HPA = 1200.0f;


constexpr float INR_ALPHA_MIN = 0.05f;
constexpr float INR_ALPHA_MAX = 0.5f;
constexpr float INR_ALPHA_DEFAULT = 0.2f;
constexpr float INR_SENSITIVITY_GAIN = 0.1f;
constexpr uint8_t INR_BUFFER_SIZE = 10;


constexpr uint8_t MAX_SENSOR_PAIRS = 8;
constexpr uint8_t MIN_SENSOR_PAIRS = 1;
constexpr uint8_t FRAME_BUFFER_SIZE = 16;


constexpr float DEFAULT_TAU_S = 30.0f;
constexpr float DEFAULT_ABSORPTIVITY = 0.90f;
constexpr float DEFAULT_SELF_HEATING_C = 0.8f;
constexpr float HC_STILL_AIR = 5.0f;
constexpr float HC_FORCED_CONV = 25.0f;


constexpr float FIAPHY_PI = 3.14159265359f;
constexpr float FIAPHY_TWO_PI = 6.28318530718f;
constexpr float FIAPHY_DEG_TO_RAD = 0.01745329251f;
constexpr float FIAPHY_RAD_TO_DEG = 57.2957795131f;

constexpr int32_t Q10_SCALE = 1024;
constexpr int32_t Q16_SCALE = 65536;

}
}

#endif
