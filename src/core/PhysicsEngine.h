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
    float lat;
    float lon;
    float altitude;

    CalibrationParams calibration;

    struct BaselineResult {
        float ghi;
        float cloudfrac;
    };
    

    BaselineResult computeReferencePath(const SensorFrame& refframe, float clearskyghi) {
        BaselineResult result;

        float humiditypart = refframe.humidity / 100.0f;

        float cloudraw = powf(humiditypart, Constants::cloudproxyexp);

        result.cloudfrac = cloudraw > 1.0f ? 1.0f :
                           (cloudraw < 0.0f ? 0.0f : cloudraw);

        float attenuation = 1.0f - Constants::kastena *
                   powf(result.cloudfrac, Constants::kastenb);

        if ( attenuation < 0.0f ) attenuation = 0.0f;

        result.ghi = clearskyghi * attenuation;

        return result;
    }
    

    float computeReactivePath(const SensorFrame& refframe,
                             const SensorFrame& fluxframe,
                             INRFilter& inrfilter,
                             float airdensity) {
        float deltat = fluxframe.tempc - refframe.tempc;

        float tempchange = inrfilter.getDerivative();

        float heatcoef = Thermodynamics::convectiveHeatTransferCoefficient(airdensity);

        float solairexcess = deltat +
                             calibration.thermaltime * tempchange -
                             calibration.selfheatoffset;

        float ghi = (heatcoef / calibration.solarabsorb) * solairexcess;

        if(ghi < 0.0f)  ghi = 0.0f;

        return ghi;
    }
    

public:
    PhysicsEngine() : lat(0), lon(0), altitude(0) {}

    void setLocation(float lat, float lon) {
        this->lat = lat;
        this->lon = lon;
    }

    void setCalibration(const CalibrationParams& cal) {
        calibration = cal;
    }
    

    RadiationResult compute(const DifferentialFrame& frame,
                           INRFilter& inrref,
                           INRFilter& inrflux) {
        RadiationResult result;

        if (!frame.valid) {
            Logger::warn("Invalid differential frame in compute()");
            return RadiationResult::invalid();
        }

        AtmosphericState atmos = Thermodynamics::calculateAtmosphericState(
            frame.ref.tempc,
            frame.ref.humidity,
            frame.ref.pressure
        );

        float filteredflux = inrflux.update(frame.flux.tempc);

        float clearskyghi = 1000.0f;

        BaselineResult baseline = computeReferencePath(frame.ref, clearskyghi);

        float reactiveghi = computeReactivePath(frame.ref, frame.flux, inrflux, atmos.airdensity);

        result.valid = true;
        result.ghi = (reactiveghi * 0.7f) + (baseline.ghi * 0.3f);
        result.baselineghi = baseline.ghi;
        result.heatflux = reactiveghi;
        result.cloudproxy = baseline.cloudfrac;
        result.tempdelta = frame.flux.tempc - frame.ref.tempc;
        result.tempchange = inrflux.getDerivative();
        result.airdensity = atmos.airdensity;
        result.vaporpressure = atmos.vaporpressure;
        result.confidence = 0.8f;
        result.timems = 0;

        return result;
    }
};

}

#endif
