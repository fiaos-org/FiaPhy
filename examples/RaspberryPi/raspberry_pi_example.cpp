/*Copyright (c) 2025 FiaOS.org. All rights reserved.
https://www.fiaos.org/open-source
*/

//==============================================================================
// FiaPhy Raspberry Pi Example
// Build: g++ -o solar_monitor raspberry_pi_example.cpp -lwiringPi -std=c++11
// Run: sudo ./solar_monitor
//==============================================================================

#include "../../src/FiaPhy.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <fstream>
#include <ctime>

// BME280 sensor simulator
class BME280Simulator {
public:
    BME280Simulator(uint8_t address) : address_(address), temp_(25.0f), 
                                       humid_(60.0f), press_(1013.25f) {}
    
    void read() {
        temp_ += (rand() % 100 - 50) / 100.0f;
        humid_ += (rand() % 100 - 50) / 100.0f;
        press_ += (rand() % 100 - 50) / 1000.0f;
        
        if(address_ == 0x77) {
            temp_ += 2.5f;
        }
    }
    
    float readTemperature() const { return temp_; }
    float readHumidity() const { return humid_; }
    float readPressure() const { return press_; }

private:
    uint8_t address_;
    float temp_, humid_, press_;
};

int main() {
    std::cout << "\n===================================================\n";
    std::cout << "   FiaPhy - Raspberry Pi Solar Radiation Monitor  \n";
    std::cout << "===================================================\n\n";
    
    FiaPhy::initialize();
    
    BME280Simulator sensor_ref(0x76);
    BME280Simulator sensor_flux(0x77);
    
    FiaPhy::SensorHub hub;
    FiaPhy::PhysicsEngine engine;
    
    if(!hub.begin(2)) {
        std::cerr << "ERROR: SensorHub initialization failed\n";
        return 1;
    }
    
    FiaPhy::CalibrationParams calib;
    calib.thermal_time_constant_s = 30.0f;
    calib.solar_absorptivity = 0.90f;
    engine.setCalibration(calib);
    
    std::ofstream log_file("solar_data.csv", std::ios::app);
    if(!log_file.is_open()) {
        std::cerr << "WARNING: Could not open log file\n";
    } else {
        log_file.seekp(0, std::ios::end);
        if(log_file.tellp() == 0) {
            log_file << "timestamp,ghi_Wm2,temp_ref_C,temp_flux_C,humidity_RH,pressure_hPa,confidence\n";
        }
    }
    
    std::cout << "Starting measurements... (Press Ctrl+C to stop)\n\n";
    std::cout << std::fixed << std::setprecision(2);
    
    uint32_t frame_count = 0;
    
    while(true) {
        auto start_time = std::chrono::steady_clock::now();
        
        sensor_ref.read();
        sensor_flux.read();
        
        hub.feedTemperature(sensor_ref.readTemperature(), 0);
        hub.feedHumidity(sensor_ref.readHumidity(), 0);
        hub.feedPressure(sensor_ref.readPressure(), 0);
        
        hub.feedTemperature(sensor_flux.readTemperature(), 1);
        hub.feedHumidity(sensor_flux.readHumidity(), 1);
        hub.feedPressure(sensor_flux.readPressure(), 1);
        
        if(hub.isFrameReady()) {
            const FiaPhy::SensorFrame* frames = hub.getFrames();
            FiaPhy::SolarFlux solar = engine.compute(frames, hub.getSensorCount());
            
            frame_count++;
            
            auto now = std::chrono::system_clock::now();
            std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
            
            std::cout << "\r[" << std::put_time(std::localtime(&timestamp), "%H:%M:%S") << "] "
                     << "GHI: " << std::setw(6) << solar.irradiance_Wm2 << " W/m² | "
                     << "T_ref: " << std::setw(5) << frames[0].temperature_C << " C | "
                     << "T_flux: " << std::setw(5) << frames[1].temperature_C << " C | "
                     << "dT: " << std::setw(5) << (frames[1].temperature_C - frames[0].temperature_C) << " C | "
                     << "Frames: " << frame_count << "   " << std::flush;
            
            if(log_file.is_open() && frame_count % 10 == 0) {
                log_file << timestamp << ","
                        << solar.irradiance_Wm2 << ","
                        << frames[0].temperature_C << ","
                        << frames[1].temperature_C << ","
                        << frames[0].humidity_RH << ","
                        << frames[0].pressure_hPa << ","
                        << solar.confidence << "\n";
                log_file.flush();
            }
            
            hub.acknowledgeFrame();
        }
        
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if(elapsed.count() < 1000) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 - elapsed.count()));
        }
    }
    
    log_file.close();
    return 0;
}

