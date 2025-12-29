#include "../../src/FiaPhy.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <fstream>
#include <ctime>

class BME280Simulator {
public:
    BME280Simulator(uint8_t address) : chip(address), temp(25.0f),
                                       humid(60.0f), press(1013.25f) {}
    
    void read() {
        temp += (rand() % 100 - 50) / 100.0f;
        humid += (rand() % 100 - 50) / 100.0f;
        press += (rand() % 100 - 50) / 1000.0f;
        
        if(chip == 0x77) {
            temp += 2.5f;
        }
    }
    
    float readTemperature() const { return temp; }
    float readHumidity() const { return humid; }
    float readPressure() const { return press; }

private:
    uint8_t chip;
    float temp, humid, press;
};

int main() {
    std::cout << "\n===================================================\n";
    std::cout << "   FiaPhy™ raspberry pi live monitor  \n";
    std::cout << "===================================================\n\n";
    
    FiaPhy::initialize();
    
    BME280Simulator sensorref(0x76);
    BME280Simulator sensorflux(0x77);
    
    FiaPhy::SensorHub hub;
    FiaPhy::PhysicsEngine engine;
    
    if ( !hub.begin(2) ) {
        std::cerr << "sensor hub init failed\n";
        return 1;
    }
    
    FiaPhy::CalibrationParams tune;
    tune.thermaltime = 30.0f;
    tune.solarabsorb = 0.90f;
    engine.setCalibration(tune);
    
    std::ofstream logfile("solar_data.csv", std::ios::app);
    if (!logfile.is_open()) {
        std::cerr << "could not open log file\n";
    } else {
        logfile.seekp(0, std::ios::end);
        if(logfile.tellp() == 0) {
            logfile << "time,ghi,tref,tflux,humidity,pressure,confidence\n";
        }
    }
    
    std::cout << "starting reads... (press Ctrl+C to stop)\n\n";
    std::cout << std::fixed << std::setprecision(2);
    
    uint32_t framecount = 0;
    
    while(true) {
        auto starttime = std::chrono::steady_clock::now();
        
        sensorref.read();
        sensorflux.read();
        
        hub.feedTemperature(sensorref.readTemperature(), 0);
        hub.feedHumidity(sensorref.readHumidity(), 0);
        hub.feedPressure(sensorref.readPressure(), 0);
        
        hub.feedTemperature(sensorflux.readTemperature(), 1);
        hub.feedHumidity(sensorflux.readHumidity(), 1);
        hub.feedPressure(sensorflux.readPressure(), 1);
        
        if(hub.isFrameReady()) {
            const FiaPhy::SensorFrame* frames = hub.getFrames();
            FiaPhy::SolarFlux solar = engine.compute(frames, hub.getSensorCount());
            
            framecount++;
            
            auto nowtime = std::chrono::system_clock::now();
            std::time_t stamp = std::chrono::system_clock::to_time_t(nowtime);
            
            std::cout << "\r[" << std::put_time(std::localtime(&stamp), "%H:%M:%S") << "] "
                     << "ghi: " << std::setw(6) << solar.irradiance << " W/m² | "
                     << "tref: " << std::setw(5) << frames[0].tempc << " C | "
                     << "tflux: " << std::setw(5) << frames[1].tempc << " C | "
                     << "dt: " << std::setw(5) << (frames[1].tempc - frames[0].tempc) << " C | "
                     << "frames: " << framecount << "   " << std::flush;
            
            if(logfile.is_open() && framecount % 10 == 0) {
                logfile << stamp << ","
                        << solar.irradiance << ","
                        << frames[0].tempc << ","
                        << frames[1].tempc << ","
                        << frames[0].humidity << ","
                        << frames[0].pressure << ","
                        << solar.confidence << "\n";
                logfile.flush();
            }
            
            hub.acknowledgeFrame();
        }
        
        auto endtime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endtime - starttime);
        
        if(elapsed.count() < 1000) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 - elapsed.count()));
        }
    }
    
    logfile.close();
    return 0;
}

