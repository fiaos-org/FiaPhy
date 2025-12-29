#ifndef FIAPHY_LOGGER_H
#define FIAPHY_LOGGER_H

#include <stdarg.h>
#include <stdio.h>

#if defined(ARDUINO)
    #include <Arduino.h>
    #define FIAPHY_PLATFORM_ARDUINO
#elif defined(ESP32)
    #include "esp_log.h"
    #define FIAPHY_PLATFORM_ESP32
#elif defined(__linux__) || defined(__unix__)
    #define FIAPHY_PLATFORM_POSIX
#else
    #define FIAPHY_PLATFORM_GENERIC
#endif

namespace FiaPhy {

enum class LogLevel {
    DEBUG = 0,
    INFO = 1,
    WARN = 2,
    ERROR = 3,
    NONE = 4
};

class Logger {
public:
    static void initialize(LogLevel minlevel = LogLevel::INFO) {
        minlevelnow = minlevel;

        #ifdef FIAPHY_PLATFORM_ARDUINO
            if ( !Serial ) {
                unsigned long start = millis();
                while ( !Serial && (millis() - start) < 1000 );
            }
        #endif

        inited = true;
    }

    static void init(LogLevel minlevel = LogLevel::INFO) {
        initialize(minlevel);
    }
    
    static void debug(const char* format, ...) {
        if (minlevelnow > LogLevel::DEBUG)  return;

        va_list args;
        va_start(args, format);
        log("[DEBUG] ", format, args);
        va_end(args);
    }

    static void info(const char* format, ...) {
        if (minlevelnow > LogLevel::INFO) return;

        va_list args;
        va_start(args, format);
        log("[INFO]  ", format, args);
        va_end(args);
    }

    static void warn(const char* format, ...) {
        if (minlevelnow > LogLevel::WARN) return;

        va_list args;
        va_start(args, format);
        log("[WARN]  ", format, args);
        va_end(args);
    }

    static void error(const char* format, ...) {
        if (minlevelnow > LogLevel::ERROR) return;

        va_list args;
        va_start(args, format);
        log("[ERROR] ", format, args);
        va_end(args);
    }

    static void setLevel( LogLevel level ) {
        minlevelnow = level;
    }

private:
    static LogLevel minlevelnow;
    static bool inited;
    
    static void log( const char* prefix, const char* format, va_list args ) {
        #ifdef FIAPHY_PLATFORM_ARDUINO
            if (Serial) {
                Serial.print(prefix);

                char buffer[128];
                vsnprintf(buffer, sizeof(buffer), format, args);
                Serial.println(buffer);
            }

        #elif defined(FIAPHY_PLATFORM_ESP32)
            char buffer[128];
            vsnprintf(buffer, sizeof(buffer), format, args);
            ESP_LOGI("FiaPhy", "%s%s", prefix, buffer);

        #else
            printf("%s", prefix);
            vprintf(format, args);
            printf("\n");
            fflush(stdout);
        #endif
    }
};


LogLevel Logger::minlevelnow = LogLevel::INFO;
bool Logger::inited = false;

}

#endif
