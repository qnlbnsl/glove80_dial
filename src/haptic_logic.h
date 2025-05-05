#ifndef HAPTIC_LOGIC_H
#define HAPTIC_LOGIC_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include "utils.h"

// Pin definitions
#define UH D1   // GPIO1 (D7)
#define UL D0   // GPIO0 (D6)
#define VH D3   // GPIO3 (D10)
#define VL D2   // GPIO2 (D8)
#define WH D9   // GPIO9 (D10)
#define WL D8  // GPIO8 (D9)

class HapticController {
public:
    HapticController();
    void setup(bool debug);
    int8_t loop(); // Returns wheel delta
    float getFilteredVelocity() const;
    float getFilteredAngle() const;
    float getLastTorque() const;
    float getDetentCenter() const;
    static float angleDiff(float a, float b);

private:
    // Constants
    static constexpr float DETENT_DEG    = 12.0f;
    static constexpr float DETENT_RAD    = DETENT_DEG * DEG2RAD;
    static constexpr float HALF_STEP     = DETENT_RAD * 0.5f;
    static constexpr float DEADBAND_RAD  = 2.5f * DEG2RAD;
    static constexpr float K_SPRING      = 3.0f;
    static constexpr float K_DAMP        = 0.15f;
    static constexpr float TORQUE_DZ     = 0.80f;
    static constexpr float KICK_VOLT     = 2.5f;  // Increased for stronger kick
    static constexpr float V_THRESH      = 0.8f;
    static constexpr float LPF_ALPHA     = 0.2f;
    static constexpr float ACC_GAIN      = 1.0f;
    static const int8_t MAX_WHEEL        = 127; // Also used by mouse logic, keep accessible? Maybe move later.
    
    // Velocity and torque control parameters
    static constexpr float VELOCITY_DEADBAND = 0.2f;  // Ignore tiny velocity oscillations
    static constexpr float VEL_SCALE_THRESHOLD = 5.0f; // Threshold where we switch from assist to damping
    static constexpr float TORQUE_DECAY = 1.0f - 0.9976f;       // Decay factor when not moving
    static constexpr unsigned long TORQUE_COOLDOWN_MS = 20;

    // FOC Objects
    BLDCMotor         motor;
    BLDCDriver6PWM    driver;
    MagneticSensorI2C sensor;

    // State variables
    float   velFilt;
    float   angleFilt;
    int32_t idxPrev;
    float   detentCtr;
    float   lastTorque;
    float   prevDirection;  // Added to store previous direction for hysteresis
    bool    torqueApplied;  // Track if torque was recently applied
    unsigned long lastTorqueTime;  // Timestamp when torque was last applied

    // Time-based cooldown - 15ms
    unsigned long currentTime = millis();

    // Helper methods
    float readAngle();
    int8_t detentStep(float angle);
    void applyHaptics(float angle);
};

#endif // HAPTIC_LOGIC_H 