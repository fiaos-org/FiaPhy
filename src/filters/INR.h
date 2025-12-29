#ifndef FIAPHY_INR_H
#define FIAPHY_INR_H

#include "../core/Types.h"
#include <math.h>

namespace FiaPhy {

class INRFilter {
public:
    INRFilter(float alphamin = 0.05f, float alphamax = 0.8f, float sensitivity = 2.0f)
        : alphaminv(alphamin), alphamaxv(alphamax), sensitivityv(sensitivity), sampledt(1.0f) {
        state.filtered = 0;
        state.prevfiltered = 0;
        state.derivative = 0;
        state.projected = 0;
        state.alpha = (alphamin + alphamax) / 2.0f;
        state.ringindex = 0;

        for (uint8_t i = 0; i < INRState::ringsize; i++) {
            state.ring[i] = 0;
        }

        initialized = false;
    }

    float update(float rawvalue, float dt = 1.0f) {
        sampledt = dt;
        if (!initialized) {
            state.filtered = rawvalue;
            state.prevfiltered = rawvalue;
            for (uint8_t i = 0; i < INRState::ringsize; i++) {
                state.ring[i] = rawvalue;
            }
            initialized = true;
            return rawvalue;
        }

        float delta = fabsf(rawvalue - state.filtered);
        state.alpha = alphaminv + sensitivityv * delta;
        if (state.alpha > alphamaxv) state.alpha = alphamaxv;
        if (state.alpha < alphaminv) state.alpha = alphaminv;

        state.prevfiltered = state.filtered;
        state.filtered = state.alpha * rawvalue +
            (1.0f - state.alpha) * state.filtered;

        state.ring[state.ringindex] = state.filtered;
        state.ringindex = (state.ringindex + 1) % INRState::ringsize;

        state.derivative = computederivative();

        return state.filtered;
    }

    float computeProjection(float thermaltime) {
        state.projected = state.filtered + thermaltime * state.derivative;
        return state.projected;
    }

    float getFiltered() const {
        return state.filtered;
    }

    float getDerivative() const{
        return state.derivative;
    }

    float getProjected() const {
        return state.projected;
    }

    float getAlpha() const {
        return state.alpha;
    }

    void reset( ) {
        initialized = false;
        state.filtered = 0;
        state.prevfiltered = 0;
        state.derivative = 0;
        state.projected = 0;
        state.alpha = (alphaminv + alphamaxv) / 2.0f;
    }

    float process(float rawvalue){
        return update(rawvalue);
    }

private:
    INRState state;
    float alphaminv;
    float alphamaxv;
    float sensitivityv;
    float sampledt;
    bool initialized;

    float computederivative() {
        uint8_t current = (state.ringindex + INRState::ringsize - 1) % INRState::ringsize;
        uint8_t previous = (current + INRState::ringsize - 2) % INRState::ringsize;

        float tprev = state.ring[previous];
        float tnext = state.ring[current];

        return (tnext - tprev) / (2.0f * sampledt);
    }
};

}

#endif
