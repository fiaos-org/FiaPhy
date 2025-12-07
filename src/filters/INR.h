/*Copyright (c) 2025 FiaOS.org. All rights reserved.
https://www.fiaos.org/open-source
*/

#ifndef FIAPHY_INR_H
#define FIAPHY_INR_H

#include "../core/Types.h"
#include <math.h>

namespace FiaPhy {

// Inertial Noise Reduction Filter
// Adaptive EMA with derivative-based thermal lag compensation
class INRFilter {
public:
    INRFilter(float alpha_min = 0.05f, float alpha_max = 0.8f, float sensitivity = 2.0f) 
        : alpha_min_(alpha_min), alpha_max_(alpha_max), sensitivity_(sensitivity), sample_dt_(1.0f) {
        state_.filtered_value = 0;
        state_.previous_filtered = 0;
        state_.derivative = 0;
        state_.projected_value = 0;
        state_.alpha = (alpha_min + alpha_max) / 2.0f;
        state_.buffer_index = 0;
        
        for(uint8_t i = 0; i < INRState::BUFFER_SIZE; i++) {
            state_.circular_buffer[i] = 0;
        }
        
        initialized_ = false;
    }
    
    // Apply adaptive EMA filter to raw input
    // dt: time since last sample in seconds (default 1.0s)
    float update(float raw_value, float dt = 1.0f) {
        sample_dt_ = dt;
        if(!initialized_) {
            state_.filtered_value = raw_value;
            state_.previous_filtered = raw_value;
            for(uint8_t i = 0; i < INRState::BUFFER_SIZE; i++) {
                state_.circular_buffer[i] = raw_value;
            }
            initialized_ = true;
            return raw_value;
        }
        
        // Adapt alpha based on signal deviation
        float delta = fabsf(raw_value - state_.filtered_value);
        state_.alpha = alpha_min_ + sensitivity_ * delta;
        if(state_.alpha > alpha_max_) state_.alpha = alpha_max_;
        if(state_.alpha < alpha_min_) state_.alpha = alpha_min_;
        
        // Apply EMA
        state_.previous_filtered = state_.filtered_value;
        state_.filtered_value = state_.alpha * raw_value + 
                               (1.0f - state_.alpha) * state_.filtered_value;
        
        // Update circular buffer
        state_.circular_buffer[state_.buffer_index] = state_.filtered_value;
        state_.buffer_index = (state_.buffer_index + 1) % INRState::BUFFER_SIZE;
        
        state_.derivative = computeDerivative();
        
        return state_.filtered_value;
    }
    
    // Compute thermal lag compensation: T_proj = T_filt + tau * dT/dt
    float computeProjection(float thermal_time_constant_s) {
        state_.projected_value = state_.filtered_value + 
                                thermal_time_constant_s * state_.derivative;
        return state_.projected_value;
    }
    
    float getFiltered() const {
        return state_.filtered_value;
    }
    
    float getDerivative() const {
        return state_.derivative;
    }
    
    float getProjected() const {
        return state_.projected_value;
    }
    
    float getAlpha() const {
        return state_.alpha;
    }
    
    void reset() {
        initialized_ = false;
        state_.filtered_value = 0;
        state_.previous_filtered = 0;
        state_.derivative = 0;
        state_.projected_value = 0;
        state_.alpha = (alpha_min_ + alpha_max_) / 2.0f;
    }
    
    float process(float raw_value) {
        return update(raw_value);
    }

private:
    INRState state_;
    float alpha_min_;
    float alpha_max_;
    float sensitivity_;
    float sample_dt_;
    bool initialized_;
    
    // Central difference derivative: (T[n+1] - T[n-1]) / (2 * dt)
    // Uses actual sample interval instead of hardcoded timing
    float computeDerivative() {
        uint8_t idx_current = (state_.buffer_index + INRState::BUFFER_SIZE - 1) % INRState::BUFFER_SIZE;
        uint8_t idx_prev = (idx_current + INRState::BUFFER_SIZE - 2) % INRState::BUFFER_SIZE;
        uint8_t idx_next = idx_current;
        
        float T_prev = state_.circular_buffer[idx_prev];
        float T_next = state_.circular_buffer[idx_next];
        
        return (T_next - T_prev) / (2.0f * sample_dt_);
    }
};

} // namespace FiaPhy

#endif // FIAPHY_INR_H
