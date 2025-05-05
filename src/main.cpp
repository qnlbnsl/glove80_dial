/*********************************************************************
 *  Haptic scroll wheel – 6° detents, acceleration, silent idle
 *  – SimpleFOC 2.3.4 –  XIAO-RP2350 + TMC6300 + AS5600
 *********************************************************************/

#include <Arduino.h>
#include <math.h>
#include "haptic_logic.h"
#include "mouse_logic.h"


HapticController hapticController;
ScrollHandler scrollHandler;

// Shared variables between cores
volatile int8_t shared_wheel = 0;
volatile float shared_velocity = 0.0f;
const bool debug = true;

// Use a simple flag as a mutex for Serial access
volatile bool serial_in_use = false;

// Function to safely print from any core
// Make it external so it can be used in other files
extern "C" void serial_println(const char* message) {
  // Simple spinlock - wait until Serial is free
  while (serial_in_use) { 
    delay(1); 
  }
  
  // Set the flag to indicate we're using Serial
  serial_in_use = true;
  
  // Print message
  Serial.println(message);
  
  // Release the lock
  serial_in_use = false;
}

void debugPrint(float vel, int8_t wheel, float angle) {
  // Reduced frequency debug printing or conditional printing might be needed
  // if Serial output becomes a bottleneck.
  // For now, keep as is.
  if (wheel == 0 && vel == 0) return; // Print only when there's activity
  Serial.print("Wheel: "); Serial.print(wheel); Serial.print("\t");
  Serial.print("Vel: "); Serial.print(vel, 2); Serial.print("\t");
  Serial.print("Angle: "); Serial.println(angle, 2);
}

// ------------------------------------------------------------------
//  SETUP Core 0
// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  // It's often recommended to wait for Serial only during debugging
  // Remove or comment out for final deployment if startup time matters
  while(!Serial) delay(3000);

  Serial.println("Starting setup");
  unsigned long start_time = millis();
  // Setup haptic controller on Core 0
  hapticController.setup(debug);
  unsigned long end_time = millis();
  Serial.print("Haptic controller setup time: ");
  Serial.println(end_time - start_time);

  Serial.println(F("Haptic scroll wheel Core 0 ready"));
  // Core 1 setup is launched automatically after setup() completes
}

// ------------------------------------------------------------------
//  SETUP Core 1
// ------------------------------------------------------------------
void setup1() {
  // Give core 0 time to initialize
  // delay(5000);
  while(!Serial) delay(3000);
  // Setup scroll handler (USB) on Core 1
  scrollHandler.setup();
  serial_println("USB Mouse Scroll Core 1 ready");
}

// ------------------------------------------------------------------
//  MAIN LOOP Core 0
// ------------------------------------------------------------------
void loop() {
  // Run haptic controller loop
  int8_t current_wheel = hapticController.loop();
  float current_velocity = hapticController.getFilteredVelocity();
  float current_angle = hapticController.getFilteredAngle();
  
  // Get the current torque using the new getter method
  float current_torque = hapticController.getLastTorque();

  // Update shared variables for Core 1
  shared_wheel = current_wheel;
  shared_velocity = current_velocity;

  // Send data in Teleplot format
  Serial.print(">velocity:");
  Serial.println(current_velocity);
  
  Serial.print(">torque:");
  Serial.println(current_torque);
  
  Serial.print(">angle:");
  Serial.println(current_angle);
  
  Serial.print(">wheel:");
  Serial.println(current_wheel);
  
  // Get the detent center using the new getter
  float detentCtr = hapticController.getDetentCenter();
  
  // Use the static angleDiff function from HapticController
  float error = HapticController::angleDiff(current_angle, detentCtr);
  Serial.print(">error:");
  Serial.println(error);

  // Small delay to prevent Serial flooding
  delay(20); // 50Hz update rate
}

// ------------------------------------------------------------------
//  MAIN LOOP Core 1
// ------------------------------------------------------------------
void loop1() {
  // Read shared variables
  int8_t wheel_for_update = shared_wheel;
  float velocity_for_update = shared_velocity;

  // Only update the scroll handler if there's new wheel input
  // or if a free scroll is currently active.
  if (wheel_for_update != 0 || scrollHandler.isFreeScrolling()) {
      scrollHandler.update(wheel_for_update, velocity_for_update);
  } else {
      // If idle (no new input and no free scroll), yield time to Core 0.
      // A small delay is often sufficient on RP2040.
      // delay(1); 
  }

  // TinyUSB device tasks run in the background automatically.

  // serial_println("[Core 1] Message");
}

