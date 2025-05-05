# Detent System Enhancement Plan

## Current System Analysis

### Hardware
- **MCU**: XIAO-RP2350 (RP2040)
- **Motor Driver**: TMC6300
- **Encoder**: AS5600 (I2C magnetic rotary encoder)
- **Power**: 5V for logic and motor

### Software
- Using SimpleFOC 2.3.4
- Current implementation has:
  - Basic detent haptics with configurable kick strength
  - Dual-core implementation (Core 0: haptics, Core 1: USB/HID)
  - Time-based cooldown for detent transitions
  - Velocity-based filtering to prevent unwanted oscillations

## Target System (motor_task) Analysis

### Key Features to Adopt
1. **PID-based Torque Control**:
   - Proportional term based on detent position
   - Derivative term that scales with detent width
   - Automatic tuning based on detent width

2. **Position Management**:
   - Min/max position boundaries with endstop behavior
   - Snap points for detent transitions
   - Sub-position tracking for fine control

3. **Idle Drift Correction**:
   - Automatic centering when idle for set time
   - EWMA filtering for velocity detection

4. **Selective Detent Positioning**:
   - Ability to define specific positions that have detents
   - Skip positions that shouldn't have detents

## Implementation Plan

### Phase 1: Preparation and Testing

1. **Create Backup**:
   - Back up current working code
   - Create a new branch for development

2. **Hardware Compatibility Check**:
   - Confirm SimpleFOC configuration for TMC6300
   - Verify AS5600 sensor implementation

### Phase 2: Core Architecture Changes

1. **Update HapticController Class**:
   ```cpp
   class HapticController {
   public:
       // Existing methods
       void setup(bool debug);
       int8_t loop();
       
       // New configuration methods
       void setConfig(const DetentConfig& config);
       void setDetentPositions(int32_t* positions, uint8_t count);
       void setMinMaxPositions(int32_t min, int32_t max);
       
       // New state tracking
       int32_t getCurrentPosition() const;
       float getSubPositionUnit() const;
       
   private:
       // New configuration struct
       struct DetentConfig {
           float position_width_radians = 12.0f * DEG2RAD;
           float detent_strength_unit = 0.5f;
           float endstop_strength_unit = 1.0f;
           float snap_point = 0.55f;
           float snap_point_bias = 0.0f;
       };
       
       // New state variables
       int32_t current_position;
       float current_detent_center;
       float latest_sub_position_unit;
       
       // Idle detection
       float idle_check_velocity_ewma;
       uint32_t last_idle_start;
       
       // Existing variables...
   };
   ```

2. **Update PID Configuration**:
   - Implement motor_task PID parameters
   - Add detent width-dependent derivative factor

### Phase 3: Implement Core Functionality

1. **Port Detent Logic** (from motor_task.cpp -> haptic_logic.cpp):
   - Angle to detent calculation
   - Snap point logic
   - Endstop handling
   - Selective detent application

2. **Port Idle Correction**:
   - EWMA velocity filtering
   - Timeout-based drift correction

3. **Enhance Torque Application**:
   - Implement PID controller for torque calculation
   - Add dead zone handling
   - Ensure velocity limiting for safety

### Phase 4: Extend API and UI Features

1. **Create Configuration Interface**:
   - Position limits
   - Detent width
   - Strength parameters
   - Custom detent positions

2. **Diagnostics and Tuning**:
   - Implement serial debug output for tuning
   - Add visualization of position/torque

## Migration Path

Rather than completely replacing the current implementation, we'll enhance it by:

1. First incorporating PID-based control
2. Then adding position management
3. Then implementing idle correction
4. Finally adding selective detent positioning

This incremental approach ensures we maintain a working system at each step.

## Code Changes

### 1. Add PID Controller

```cpp
// Add to haptic_logic.h
private:
    // PID controller for torque
    float PID_P = 4.0f;
    float PID_I = 0.0f;
    float PID_D = 0.04f;
    float PID_output_ramp = 10000;
    float PID_limit = 10.0f;
    
    // PID state variables
    float pid_error_prev = 0.0f;
    float pid_output_prev = 0.0f;
    float pid_integral = 0.0f;
    
    // PID calculation
    float calculatePID(float error);
```

### 2. Position Management

```cpp
// Add to haptic_logic.cpp
float HapticController::calculatePositionChange(float angle_to_detent_center) {
    float snap_point_radians = config.position_width_radians * config.snap_point;
    
    // Determine if we need to snap to next/prev detent
    if (angle_to_detent_center > snap_point_radians && 
        (config.min_position <= config.max_position || current_position > config.min_position)) {
        return config.position_width_radians; // Move one detent
    } else if (angle_to_detent_center < -snap_point_radians && 
              (config.min_position <= config.max_position || current_position < config.max_position)) {
        return -config.position_width_radians; // Move one detent
    }
    
    return 0.0f; // Stay in current detent
}
```

### 3. Torque Application Logic

```cpp
// Replace in applyHaptics
float torque = 0.0f;

// Calculate error from detent center
float error = angle_to_detent_center;

// Apply dead zone adjustment
float dead_zone_adjustment = applyDeadZone(error);
error -= dead_zone_adjustment;

// Check for out-of-bounds condition
bool out_of_bounds = isOutOfBounds(error);

// Apply appropriate PID parameters
setPIDParameters(out_of_bounds);

// Calculate torque via PID
torque = calculatePID(-error);

// Apply velocity limiting
if (fabsf(velFilt) > 60.0f) {
    torque = 0.0f;
}

// Apply torque
motor.move(torque);
```

## Testing Milestones

1. **Base PID Implementation**:
   - Test basic PID control with fixed detent position
   - Verify torque response to different detent widths

2. **Position Tracking**:
   - Test position increment/decrement on detent crossing
   - Verify min/max boundary enforcement

3. **Detent Customization**:
   - Test selective detent application
   - Validate different detent strengths

4. **Complete Integration**:
   - Test full system under various user scenarios
   - Optimize parameters for best feel

## Conclusion

This approach preserves the best aspects of both systems:
- The current system's dual-core architecture and USB HID integration
- The motor_task's sophisticated detent management and PID control

The result will be a highly configurable haptic dial with crisp detents, natural motion, and precise position tracking. 