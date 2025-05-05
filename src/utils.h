#ifndef UTILS_H
#define UTILS_H

#include <math.h> // Include for PI if not defined elsewhere (like Arduino.h)

// Helper macro for converting degrees to radians
#ifndef DEG2RAD
  #define DEG2RAD (M_PI / 180.0f)
#endif

// Helper macro for converting radians to degrees
#ifndef RAD2DEG
  #define RAD2DEG (180.0f / M_PI)
#endif

// inline float map_range(float x, float in_min, float in_max, float out_min, float out_max) {
//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }

#endif // UTILS_H 