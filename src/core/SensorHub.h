#ifndef FIAPHY_SENSORHUB_H
#define FIAPHY_SENSORHUB_H

#include "Types.h"
#include "Constants.h"
#include "../platform/Logger.h"
#include <stdint.h>

namespace FiaPhy {

class SensorHub {
private:
    SensorFrame refbuffer[Constants::maxsensorpairs];
    SensorFrame fluxbuffer[Constants::maxsensorpairs];

    uint8_t refsensormask;
    uint8_t fluxsensormask;

    uint8_t reftempcount;
    uint8_t refhumiditycount;
    uint8_t refpressurecount;
    uint8_t fluxtempcount;
    uint8_t fluxhumiditycount;
    uint8_t fluxpressurecount;

    float prevreftemp[Constants::maxsensorpairs];
    float prevrefhumidity[Constants::maxsensorpairs];
    float prevrefpressure[Constants::maxsensorpairs];
    float prevfluxtemp[Constants::maxsensorpairs];
    float prevfluxhumidity[Constants::maxsensorpairs];
    float prevfluxpressure[Constants::maxsensorpairs];

    bool prevready[Constants::maxsensorpairs * 2];

    bool frameready;
    uint32_t framecounter;
    

    bool isUnrealisticJump(float newval, float prevval, float maxjump,
                           uint8_t sensorid, bool initialized) {
        if (!initialized) {
            return false;
        }
        
        float delta = newval > prevval ? (newval - prevval) : (prevval - newval);
        
        if (delta > maxjump) {
            Logger::warn("Unrealistic jump detected on sensor");
            return true;
        }
        
        return false;
    }
    

    bool isValidRange(float tempc, float humidity, float pressure) {
         return tempc >= Constants::minvalidtemp &&
             tempc <= Constants::maxvalidtemp &&
             humidity >= Constants::minvalidhumidity &&
             humidity <= Constants::maxvalidhumidity &&
             pressure >= Constants::minvalidpressure &&
             pressure <= Constants::maxvalidpressure;
    }
    

    void   checkFrameCompletion() {
        uint8_t refcomplete = 0;
        uint8_t fluxcomplete = 0;
        
        for (uint8_t i = 0; i < Constants::maxsensorpairs; i++) {
            if ((refsensormask & (1 << i)) && refbuffer[i].complete) {
                refcomplete++;
            }
            if ((fluxsensormask & (1 << i)) && fluxbuffer[i].complete) {
                fluxcomplete++;
            }
        }
        
        uint8_t refactive = countBits(refsensormask);
        uint8_t fluxactive = countBits(fluxsensormask);
        
        frameready = (refcomplete == refactive) &&
                     (fluxcomplete == fluxactive) &&
                     (refactive > 0) && (fluxactive > 0);
    }
    

    uint8_t countBits( uint8_t value ) const {
        uint8_t count = 0;
        while (value) {
            count += value & 1;
            value >>= 1;
        }
        return count;
    }
    
public:
    SensorHub() :
        refsensormask(0), fluxsensormask(0),
        reftempcount(0), refhumiditycount(0), refpressurecount(0),
        fluxtempcount(0), fluxhumiditycount(0), fluxpressurecount(0),
        frameready(false), framecounter(0) {
        
        for (uint8_t i = 0; i < Constants::maxsensorpairs; i++) {
            refbuffer[i].reset();
            fluxbuffer[i].reset();
            prevready[i] = false;
            prevready[i + Constants::maxsensorpairs] = false;
        }
    }
    

    void feedTemperature(SensorType type, uint8_t sensorid, float tempc) {
        if (sensorid >= Constants::maxsensorpairs) {
            Logger::error("Sensor ID exceeds maximum");
            return;
        }
        
        SensorFrame* buffer = (type == SensorType::REFERENCE) ? refbuffer : fluxbuffer;
        uint8_t* sensormask = (type == SensorType::REFERENCE) ? &refsensormask : &fluxsensormask;
        uint8_t* tempcount = (type == SensorType::REFERENCE) ? &reftempcount : &fluxtempcount;
        float* prevtemp = (type == SensorType::REFERENCE) ? prevreftemp : prevfluxtemp;
        uint8_t previdx = (type == SensorType::REFERENCE) ? sensorid : (sensorid + Constants::maxsensorpairs);
        
        if (isUnrealisticJump(tempc, prevtemp[sensorid], Constants::maxtempjump,
                              sensorid, prevready[previdx])) {
            Logger::error("Temperature jump validation failed");
            return;
        }
        
        buffer[sensorid].tempc = tempc;
        buffer[sensorid].hastemp = true;
        buffer[sensorid].sensorid = sensorid;
        prevtemp[sensorid] = tempc;
        prevready[previdx] = true;
        
        *sensormask |= (1 << sensorid);
        
        if (!buffer[sensorid].hashumidity && !buffer[sensorid].haspressure) {
            (*tempcount)++;
        }
        
        buffer[sensorid].checkCompletion();
        checkFrameCompletion();
    }
    

    void feedHumidity(SensorType type, uint8_t sensorid, float humidityval) {
        if (sensorid >= Constants::maxsensorpairs) {
            Logger::error("Sensor ID exceeds maximum");
            return;
        }
        
        SensorFrame* buffer = (type == SensorType::REFERENCE) ? refbuffer : fluxbuffer;
        uint8_t* sensormask = (type == SensorType::REFERENCE) ? &refsensormask : &fluxsensormask;
        uint8_t* humiditycount = (type == SensorType::REFERENCE) ? &refhumiditycount : &fluxhumiditycount;
        float* prevhumidity = (type == SensorType::REFERENCE) ? prevrefhumidity : prevfluxhumidity;
        uint8_t previdx = (type == SensorType::REFERENCE) ? sensorid : (sensorid + Constants::maxsensorpairs);
        
        if (isUnrealisticJump(humidityval, prevhumidity[sensorid],
                              Constants::maxhumidityjump,
                              sensorid, prevready[previdx])) {
            Logger::error("Humidity jump validation failed");
            return;
        }
        
        buffer[sensorid].humidity = humidityval;
        buffer[sensorid].hashumidity = true;
        buffer[sensorid].sensorid = sensorid;
        prevhumidity[sensorid] = humidityval;
        prevready[previdx] = true;
        
        *sensormask |= (1 << sensorid);
        
        if (!buffer[sensorid].hastemp && !buffer[sensorid].haspressure) {
            (*humiditycount)++;
        }
        
        buffer[sensorid].checkCompletion();
        checkFrameCompletion();
    }
    

    void feedPressure(SensorType type, uint8_t sensorid, float pressureval) {
        if (sensorid >= Constants::maxsensorpairs) {
            Logger::error("Sensor ID exceeds maximum");
            return;
        }
        
        SensorFrame* buffer = (type == SensorType::REFERENCE) ? refbuffer : fluxbuffer;
        uint8_t* sensormask = (type == SensorType::REFERENCE) ? &refsensormask : &fluxsensormask;
        uint8_t* pressurecount = (type == SensorType::REFERENCE) ? &refpressurecount : &fluxpressurecount;
        float* prevpressure = (type == SensorType::REFERENCE) ? prevrefpressure : prevfluxpressure;
        uint8_t previdx = (type == SensorType::REFERENCE) ? sensorid : (sensorid + Constants::maxsensorpairs);
        
        if (isUnrealisticJump(pressureval, prevpressure[sensorid],
                              Constants::maxpressurejump,
                              sensorid, prevready[previdx])) {
            Logger::error("Pressure jump validation failed");
            return;
        }
        
        buffer[sensorid].pressure = pressureval;
        buffer[sensorid].haspressure = true;
        buffer[sensorid].sensorid = sensorid;
        prevpressure[sensorid] = pressureval;
        prevready[previdx] = true;
        
        *sensormask |= (1 << sensorid);
        
        if (!buffer[sensorid].hastemp && !buffer[sensorid].hashumidity) {
            (*pressurecount)++;
        }
        
        buffer[sensorid].checkCompletion();
        checkFrameCompletion();
    }
    

    bool   isFrameReady() const {
        return frameready;
    }
    

    DifferentialFrame  getDifferentialFrame() {
        DifferentialFrame result;
        
        if (!frameready) {
            Logger::warn("getDifferentialFrame() called before frame ready");
            return result;
        }

        for (uint8_t i = 0; i < Constants::maxsensorpairs; i++) {
            if ((refsensormask & (1 << i)) && refbuffer[i].complete &&
                (fluxsensormask & (1 << i)) && fluxbuffer[i].complete) {

                result.ref = refbuffer[i];
                result.flux = fluxbuffer[i];
                result.valid = true;
                break;
            }
        }

        framecounter++;
        reset();

        return result;
    }
    

    SensorFrame  getFrame() {
        if (!frameready) {
            Logger::warn("getFrame() called before frame ready");
        }
        
        SensorFrame result;
        
        for (uint8_t i = 0; i < Constants::maxsensorpairs; i++) {
            if ((refsensormask & (1 << i)) && refbuffer[i].complete) {
                result = refbuffer[i];
                break;
            }
        }
        
        framecounter++;
        reset();
        
        return result;
    }
    
    ValidationResult   validateFrame(const SensorFrame& frame) {
        ValidationResult result;

        if (!isValidRange(frame.tempc, frame.humidity, frame.pressure)) {
            result.valid = false;
            result.errorcode = ErrorCode::VALUE_OUT_OF_RANGE;
            result.message = "Sensor value outside valid range";
            Logger::error(result.message);
            return result;
        }

        result.valid = true;
        result.errorcode = ErrorCode::OK;
        result.message = "Validation passed";
        return result;
    }
    

    uint8_t getSensorCount(SensorType type) const {
        return countBits(type == SensorType::REFERENCE ? refsensormask : fluxsensormask);
    }

    void reset() {
        refsensormask = 0;
        fluxsensormask = 0;
        reftempcount = 0;
        refhumiditycount = 0;
        refpressurecount = 0;
        fluxtempcount = 0;
        fluxhumiditycount = 0;
        fluxpressurecount = 0;
        frameready = false;

        for (uint8_t i = 0; i < Constants::maxsensorpairs; i++) {
            refbuffer[i].reset();
            fluxbuffer[i].reset();
        }
    }
};

}

#endif
