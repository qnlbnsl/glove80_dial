#include "haptic_logic.h"
#include "utils.h" // Include for DEG2RAD
#include "hardware/sync.h"


// Constructor: Initialize FOC objects and state variables
HapticController::HapticController()
    : motor(7), // 7 pole pairs
      driver(UH, UL, VH, VL, WH, WL),
      sensor(AS5600_I2C),
      velFilt(0.0f),
      angleFilt(0.0f),
      idxPrev(0),
      detentCtr(0.0f),
      lastTorque(0.0f),
      prevDirection(0.0f),
      torqueApplied(false),
      lastTorqueTime(0)
{
}

// Utility: wrap-around difference
float HapticController::angleDiff(float a, float b) {
  float d = a - b;
  while(d >  PI) d -= _2PI;
  while(d < -PI) d += _2PI;
  return d;
}

// 1. Read & low-pass filter angle + update velocity filter
float HapticController::readAngle() {
  uint32_t save = save_and_disable_interrupts();
  sensor.update();
  float raw = sensor.getAngle();
  // angle LPF (smooth small jumps)
  angleFilt += LPF_ALPHA * angleDiff(raw, angleFilt);
  // velocity LPF
  velFilt   += LPF_ALPHA * (motor.shaft_velocity - velFilt);
  restore_interrupts(save);
  return angleFilt;
}

// 2. Hysteresis‐based detent stepping
int8_t HapticController::detentStep(float angle) {
  // 1) ignore tiny motion
  if (fabsf(velFilt) < V_THRESH) return 0;

  // 2) hysteresis slot crossing
  float err = angleDiff(angle, detentCtr);
  int32_t diff = 0;
  if      (err >  HALF_STEP + DEADBAND_RAD) diff = +1;
  else if (err < -HALF_STEP - DEADBAND_RAD) diff = -1;
  else                                     return 0;

  // 3) logarithmic accel mapping
  float v = fabsf(velFilt);
  // log1p(v) = ln(1 + v), smooth compression of large v
  int32_t accel = (int32_t)(ACC_GAIN * log1p(v));
  accel = min(accel, (int32_t)MAX_WHEEL);

  // 4) build wheel delta
  int32_t wheel = diff * (1 + accel);

  // 5) commit new slot
  idxPrev   += diff;
  detentCtr  = idxPrev * DETENT_RAD;

  // 6) clamp and return
  return (int8_t)constrain(wheel, -MAX_WHEEL, MAX_WHEEL);
}

// 3. Smoothed haptic torque application
void HapticController::applyHaptics(float angle) {
  // Calculate error from nearest detent
  float err = angleDiff(angle, detentCtr);
  float torque = 0.0f;
  
  // Apply deadband for tiny velocity oscillations
  float filteredVel = (fabsf(velFilt) < VELOCITY_DEADBAND) ? 0.0f : velFilt;
  
  // Determine if we're moving significantly
  bool moving = fabsf(filteredVel) > V_THRESH;
  
  // Update current time
  currentTime = millis();
  
  // Check if cooldown period has elapsed
  if (torqueApplied && (currentTime - lastTorqueTime) >= TORQUE_COOLDOWN_MS) {
    torqueApplied = false;
  }
  
  // DETENT-BASED HAPTIC FEEDBACK WITH DIRECT TORQUE APPLICATION
  if (moving) {
    // Calculate normalized position within detent (0 = at detent, 1 = halfway to next)
    float normalizedPos = fabsf(err) / HALF_STEP;
    
    // At detent center, apply no torque (snap-in effect)
    if (fabsf(err) < DEADBAND_RAD) {
      torque = 0;
    } 
    // Moving away from detent - apply force to pull back to detent
    else {
      // Direction of error (which way to pull)
      float errorDirection = (err > 0) ? 1.0f : -1.0f;
      
      // Spring-like force that increases as we move away from detent
      // with slight damping based on velocity direction
      float springForce = K_SPRING * fabsf(err);
      
      // Add small damping component in direction of motion to help settling
      float dampingComp = (filteredVel != 0) ? 
                          (K_DAMP * filteredVel * ((filteredVel * err > 0) ? 1.0f : 0.5f)) : 
                          0.0f;
      
      // Calculate final torque - spring pulls toward detent, damping adds stability
      torque = errorDirection * springForce - dampingComp;
      
      // Apply kick at transition points for more distinct detent feel
      // Use a narrower transition zone for more focused kick (75% to 95% of half step)
      float transitionStartPoint = HALF_STEP * 0.75f;
      float transitionEndPoint = HALF_STEP * 0.95f;
      
      if (!torqueApplied && fabsf(err) > transitionStartPoint && fabsf(err) < transitionEndPoint) {
        // Apply stronger kick in direction toward next detent when in transition zone
        // Scale kick strength based on velocity for more natural feel at different speeds
        float velocityScale = min(1.0f, fabsf(filteredVel) / 10.0f + 0.5f);
        float kickStrength = KICK_VOLT * velocityScale;
        
        // Direction toward next detent
        float kickDirection = (err > 0) ? 1.0f : -1.0f;
        
        // Apply the kick as an additional impulse on top of the spring torque
        torque += kickDirection * kickStrength;
        
        // Set cooldown after applying transition kick
        torqueApplied = true;
        lastTorqueTime = currentTime;
        
        Serial.print(">kickStrength:");
        Serial.println(kickStrength);
      }
      
      // Check if torque is opposing movement direction and zero it if so
      // This prevents fighting against user's intended motion
      if (filteredVel != 0 && (torque * filteredVel < 0)) {
        // Torque and velocity have opposite signs - don't oppose user movement
        torque = 0;
      }
    }
    
    // Constrain to motor limits but ensure we're using enough voltage for a solid kick
    torque = constrain(torque, -motor.voltage_limit, motor.voltage_limit);
    
    // No LPF - direct torque application for crisp detents
    lastTorque = torque;
  } else {
    // When not moving, immediately set torque to zero for crisp detents
    lastTorque = 0.0f;
  }
  
  Serial.print(">directTorque:");
  Serial.println(lastTorque);
  motor.move(lastTorque);
}

// Public setup method
void HapticController::setup(bool debug) {
  if (debug) {
    Serial.println("HapticController setup");
  }
  // driver config
  if (debug) {
    Serial.println("Driver config");
  }
  driver.voltage_power_supply = 5.0f;
  driver.voltage_limit        = 5.0f;
  driver.pwm_frequency        = 32000;
  
  if (!driver.init()) {
    if (debug) {
      Serial.println("Driver init failed");
    }
  }
  
  // link driver & sensor
  motor.linkDriver(&driver);
  sensor.init();
  if (debug) {
    Serial.println("Sensor init");
  }
  motor.linkSensor(&sensor);
  if (debug) {
    Serial.println("Linked sensor");
  }
  // control modes
  motor.controller        = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.voltage_limit     = 3.0f;
  if (debug) {
    Serial.print("Voltage limit set to: ");
    Serial.println(motor.voltage_limit, 2);
  }
  // initialize FOC
  if (debug) Serial.println("Attempting motor.init()...");
  motor.init();
  if (debug) Serial.println("motor.init() completed.");
  
  if (debug) Serial.println("Attempting motor.initFOC()...");
  if (!motor.initFOC()) {
    Serial.println("!!! FOC init failed !!!");
    // Consider adding a loop here or other error handling if FOC init is critical
    // while(1); 
  }
  if (debug) Serial.println("motor.initFOC() completed.");
  
  // --- current‐loop PID tuning (d/q axes) ---
  motor.PID_current_q.P = 0.5f;
  motor.PID_current_q.I = 0.1f;
  motor.PID_current_d.P = 0.5f;
  motor.PID_current_d.I = 0.1f;
  // leave D terms at 0

  // establish initial detent
  float initA = sensor.getAngle(); // Need sensor initialized first
  idxPrev   = (int32_t)roundf(initA / DETENT_RAD);
  detentCtr = idxPrev * DETENT_RAD;
  angleFilt = detentCtr;   // start smooth angle at centre

  // At start of haptic_logic.cpp setup
  // Make sure this core owns the hardware
  i2c_init(i2c0, 400000);
  gpio_set_function(D4, GPIO_FUNC_I2C);
  gpio_set_function(D5, GPIO_FUNC_I2C);
}

// Public loop method
int8_t HapticController::loop() {
  motor.loopFOC();
  float currentAngle = readAngle();
  int8_t wheelDelta = detentStep(currentAngle);
  applyHaptics(currentAngle);
  return wheelDelta;
}

// Getters
float HapticController::getFilteredVelocity() const {
    return velFilt;
}

float HapticController::getFilteredAngle() const {
    return angleFilt;
}

float HapticController::getLastTorque() const {
    return lastTorque;
}

float HapticController::getDetentCenter() const {
    return detentCtr;
} 