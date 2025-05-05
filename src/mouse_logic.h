#ifndef MOUSE_LOGIC_H
#define MOUSE_LOGIC_H

#include <Arduino.h>
// #include <Mouse.h>
#include <Adafruit_TinyUSB.h>

// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] = {
  TUD_HID_REPORT_DESC_MOUSE()
};

// Butterworth low-pass filter coefficients
typedef struct {
  float b0, b1, b2, a1, a2;
} butterworth_coeffs_t;

class ScrollHandler {
public:
    ScrollHandler();
    void setup();
    void update(int8_t wheel_delta, float velocity);
    bool isFreeScrolling() const;

private:
    // HID instance - Correctly configured for USB Device mode
    Adafruit_USBD_HID usb_hid;
    
    // Send a scroll wheel event
    void scroll(int8_t wheel_delta);

    // Butterworth filter functions
    butterworth_coeffs_t butterworth_lowpass(float cutoff_frequency, float sampling_frequency);
    float butterworth_filter(float data, butterworth_coeffs_t *coeffs, float *filtered, float *prev1, float *prev2);
    void filter_wheel(int8_t &wheel_delta);

    // Filter settings
    static constexpr float SAMPLING_FREQUENCY = 100.0f;  // Hz
    static constexpr float CUTOFF_FREQUENCY = 10.0f;     // Hz
    butterworth_coeffs_t wheel_coeffs;
    float filtered_wheel = 0.0f;
    float prev1_wheel = 0.0f;
    float prev2_wheel = 0.0f;

    // Constants
    static constexpr float FREE_FRICTION = 0.9976f;
    static constexpr float FREE_DECEL   = 0.2f;    // Alternative friction method (commented out in original)
    static constexpr float FREE_THRESH     = 6.0f;
    static constexpr unsigned long FREE_INTERVAL = 20; // ms
    static constexpr int8_t MAX_WHEEL    = 127; // Copied from HapticController for now

    // State variables
    bool     freeScrollActive;
    float    freeMomentum;
    unsigned long freeLastTime;
};

#endif // MOUSE_LOGIC_H 