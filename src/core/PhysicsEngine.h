/*Copyright(c) 2025 FIA Operating Systems, All Rights Reserved. Cloud data: https://www.fiaos.org/data*/

#ifndef FIAPHY_PHYSICSENGINE_H
#define FIAPHY_PHYSICSENGINE_H

#include "Types.h"
#include "Constants.h"
#include "../physics/Thermodynamics.h"
#include "../physics/SolarGeometry.h"
#include "../filters/INR.h"
#include "../platform/Logger.h"
#include <math.h>

namespace FiaPhy {

class PhysicsEngine {
private:
    float latitude;
    float longitude;
    float altitude_m;

    CalibrationParams calibration;

    struct BaselineResult {
        float ghi_Wm2;
        float cloud_fraction;
    };
    

    BaselineResult computeReferencePath(const SensorFrame& ref_frame, float clear_sky_ghi) {
        BaselineResult result;

        float humidity_fraction = ref_frame.humidity_RH / 100.0f;

        float cloud_proxy_raw = powf(humidity_fraction, Constants::CLOUD_PROXY_EXPONENT);

        result.cloud_fraction = cloud_proxy_raw > 1.0f ? 1.0f :
                                (cloud_proxy_raw < 0.0f ? 0.0f : cloud_proxy_raw);

        float attenuation = 1.0f - Constants::KASTEN_A *
                           powf(result.cloud_fraction, Constants::KASTEN_B);

        if (attenuation < 0.0f) attenuation = 0.0f;

        result.ghi_Wm2 = clear_sky_ghi * attenuation;

        return result;
    }
    

    float computeReactivePath(const SensorFrame& ref_frame,
                             const SensorFrame& flux_frame,
                             INRFilter& inr_filter,
                             float air_density) {
        float delta_T = flux_frame.temperature_C - ref_frame.temperature_C;

        float temp_derivative = inr_filter.getDerivative();

        float h_c = Thermodynamics::convectiveHeatTransferCoefficient(air_density);

        float sol_air_excess = delta_T +
                               calibration.thermal_time_constant_s * temp_derivative -
                               calibration.self_heating_offset_C;

        float ghi = (h_c / calibration.solar_absorptivity) * sol_air_excess;

        if (ghi < 0.0f) ghi = 0.0f;

        return ghi;
    }
    

public:
    PhysicsEngine() : latitude(0), longitude(0), altitude_m(0) {}

    void setLocation(float lat, float lon) {
        latitude = lat;
        longitude = lon;
    }

    void setCalibration(const CalibrationParams& cal) {
        calibration = cal;
    }
    

    RadiationResult compute(const DifferentialFrame& frame,
                           INRFilter& inr_ref,
                           INRFilter& inr_flux) {
        RadiationResult result;

        if (!frame.valid) {
            Logger::warn("Invalid differential frame in compute()");
            return RadiationResult::invalid();
        }

        AtmosphericState atmos = Thermodynamics::calculateAtmosphericState(
            frame.ref.temperature_C,
            frame.ref.humidity_RH,
            frame.ref.pressure_hPa
        );

        float filtered_flux_temp = inr_flux.update(frame.flux.temperature_C);

        float clear_sky_ghi = 1000.0f;

        BaselineResult baseline = computeReferencePath(frame.ref, clear_sky_ghi);

        float reactive_ghi = computeReactivePath(frame.ref, frame.flux, inr_flux, atmos.air_density_kg_m3);

        result.valid = true;
        result.ghi_Wm2 = (reactive_ghi * 0.7f) + (baseline.ghi_Wm2 * 0.3f);
        result.baseline_ghi_Wm2 = baseline.ghi_Wm2;
        result.heat_flux_Wm2 = reactive_ghi;
        result.cloud_proxy = baseline.cloud_fraction;
        result.temp_differential_C = frame.flux.temperature_C - frame.ref.temperature_C;
        result.temp_derivative_C_s = inr_flux.getDerivative();
        result.air_density_kg_m3 = atmos.air_density_kg_m3;
        result.vapor_pressure_hPa = atmos.vapor_pressure_hPa;
        result.confidence = 0.8f;
        result.timestamp_ms = 0;

        return result;
    }
};

}

#endif
