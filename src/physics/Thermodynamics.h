/*Copyright(c) 2025 FIA Operating Systems, All Rights Reserved. Cloud data: https://www.fiaos.org/data*/

#ifndef FIAPHY_THERMODYNAMICS_H
#define FIAPHY_THERMODYNAMICS_H

#include "../core/Types.h"
#include <math.h>

namespace FiaPhy {
namespace Thermodynamics {


inline float saturationVaporPressure(float temp_C) {
    float temp_K = temp_C + 273.15f;
    
    float numerator = 17.67f * (temp_K - 273.15f);
    float denominator = temp_K - 29.65f;
    
    return 6.112f * expf(numerator / denominator);
}


inline float vaporPressure(float temp_C, float humidity_RH) {
    float e_sat = saturationVaporPressure(temp_C);
    return e_sat * (humidity_RH / 100.0f);
}


inline float airDensity(float pressure_hPa, float temp_C, float humidity_RH) {
    float temp_K = temp_C + 273.15f;
    float vapor_pressure_hPa = vaporPressure(temp_C, humidity_RH);
    
    float pressure_Pa = pressure_hPa * 100.0f;
    float vapor_pressure_Pa = vapor_pressure_hPa * 100.0f;

    float dry_density = (pressure_Pa - vapor_pressure_Pa) /
                       (Constants::R_DRY_AIR * temp_K);

    float vapor_density = vapor_pressure_Pa /
                         (Constants::R_WATER_VAPOR * temp_K);
    
    return dry_density + vapor_density;
}


inline float mixingRatio(float pressure_hPa, float vapor_pressure_hPa) {
    return 0.622f * vapor_pressure_hPa / (pressure_hPa - vapor_pressure_hPa);
}


inline float specificEnthalpy(float temp_C, float mixing_ratio) {
    float h_dry = 1.006f * temp_C;
    float h_vapor = 2501.0f + 1.86f * temp_C;
    return h_dry + mixing_ratio * h_vapor;
}


inline AtmosphericState calculateAtmosphericState(float temp_C, float humidity_RH, float pressure_hPa) {
    AtmosphericState state;

    state.temperature_K = temp_C + 273.15f;
    state.saturation_pressure_hPa = saturationVaporPressure(temp_C);
    state.vapor_pressure_hPa = vaporPressure(temp_C, humidity_RH);

    state.air_density_kg_m3 = airDensity(pressure_hPa, temp_C, humidity_RH);

    state.mixing_ratio = mixingRatio(pressure_hPa, state.vapor_pressure_hPa);

    state.specific_enthalpy_kJ_kg = specificEnthalpy(temp_C, state.mixing_ratio);
    
    return state;
}


inline float convectiveHeatTransferCoefficient(float air_density_kg_m3) {
    constexpr float h_c_sea_level = 10.0f;
    constexpr float rho_sea_level = 1.225f;

    return h_c_sea_level * sqrtf(air_density_kg_m3 / rho_sea_level);
}


}
}

#endif
