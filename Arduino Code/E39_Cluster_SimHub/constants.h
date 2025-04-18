#ifndef CONSTANTS_H
#define CONSTANTS_H

// RPM gauge range and conversion
#define MAX_RPM        7000
#define MIN_RPM        0
#define RPM_GAIN       6.1786
#define RPM_OFFSET     1321

// Temperature range (used in updateTemperatureGauge)
#define MIN_TEMP       30   // Minimum realistic coolant temperature
#define MAX_TEMP       130  // Maximum before triggering overheat

// Speed limits (in km/h)
#define MIN_SPEED_KMH  6    // ~4 mph
#define MAX_SPEED_KMH  263  // ~163 mph

#endif