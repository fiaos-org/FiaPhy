#ifndef FIAPHY_CONSTANTS_H
#define FIAPHY_CONSTANTS_H

namespace FiaPhy {
namespace   Constants  {

constexpr float rdryair = 287.058f;
constexpr float rwatervapor = 461.495f;
constexpr float epsilon = 0.622f;

constexpr float zerocelsius = 273.15f;
constexpr float stefanboltzmann = 5.67e-8f;

constexpr float magnusa = 17.67f;
constexpr float magnusb = 29.65f;
constexpr float magnusc = 6.112f;

constexpr float cpdryair = 1.006f;
constexpr float latentheat = 2501.0f;
constexpr float cpwatervapor = 1.86f;

constexpr float kastena = 0.75f;
constexpr float kastenb = 3.4f;
constexpr float cloudproxyexp = 1.8f;

constexpr float solarconstant = 1367.0f;
constexpr float maxghisealevel = 1200.0f;

constexpr float sensorarea = 6.25e-6f;
constexpr float sensorheatcapacity = 0.5f;
constexpr float tempresolution = 0.01f;
constexpr float humidityresolution = 0.08f;
constexpr float pressureresolution = 0.18f;

constexpr float maxtempjump = 5.0f;
constexpr float maxhumidityjump = 20.0f;
constexpr float maxpressurejump = 10.0f;

constexpr float minvalidtemp = -100.0f;
constexpr float maxvalidtemp = 100.0f;
constexpr float minvalidhumidity = 0.0f;
constexpr float maxvalidhumidity = 100.0f;
constexpr float minvalidpressure = 300.0f;
constexpr float maxvalidpressure = 1200.0f;

constexpr float inralphamin = 0.05f;
constexpr float inralphamax = 0.5f;
constexpr float inralphadefault = 0.2f;
constexpr float inrsensitivity = 0.1f;
constexpr uint8_t inrbuffersize = 10;

constexpr uint8_t maxsensorpairs = 8;
constexpr uint8_t minsensorpairs = 1;
constexpr uint8_t framebuffersize = 16;

constexpr float defaulttau = 30.0f;
constexpr float defaultabsorptivity = 0.90f;
constexpr float defaultselfheat = 0.8f;
constexpr float hcstillair = 5.0f;
constexpr float hcforcedconv = 25.0f;

constexpr float pi = 3.14159265359f;
constexpr float twopi = 6.28318530718f;
constexpr float degtorad = 0.01745329251f;
constexpr float radtodeg = 57.2957795131f;

constexpr int32_t q10scale = 1024;
constexpr int32_t q16scale = 65536;

}
}

#endif
