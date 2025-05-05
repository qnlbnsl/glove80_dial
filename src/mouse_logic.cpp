#include "mouse_logic.h"
#include <math.h> // For fabsf, roundf

// Declare the external serial_println function
extern "C" void serial_println(const char* message);

ScrollHandler::ScrollHandler()
    : usb_hid(desc_hid_report, sizeof(desc_hid_report), HID_ITF_PROTOCOL_MOUSE, 2, false),
      freeScrollActive(false),
      freeMomentum(0.0f),
      freeLastTime(0)
{
}

void ScrollHandler::setup() {
    serial_println("[Core 1] ScrollHandler::setup() started.");
    // Set up TinyUSB HID Device
    serial_println("[Core 1] Attempting usb_hid.begin()...");
    usb_hid.begin();
    serial_println("[Core 1] usb_hid.begin() completed.");
    
    // Initialize the Butterworth filter coefficients
    wheel_coeffs = butterworth_lowpass(CUTOFF_FREQUENCY, SAMPLING_FREQUENCY);

    // Wait for USB device to be mounted by the Host (Computer)
    serial_println("[Core 1] Waiting for TinyUSBDevice.mounted()...");
    while (!TinyUSBDevice.mounted()) {
        delay(1);
    }
    serial_println("[Core 1] TinyUSBDevice is mounted.");
    
    serial_println("[Core 1] USB HID Mouse Scroll device ready");
}

// ScrollHandler::loop() removed - TinyUSB device runs its tasks internally

void ScrollHandler::scroll(int8_t wheel_delta) {
    hid_mouse_report_t report = {
        .buttons = 0,
        .x = 0,
        .y = 0,
        .wheel = wheel_delta,
        .pan = 0
    };
    usb_hid.sendReport(0, &report, sizeof(report));
}

void ScrollHandler::update(int8_t wheel_delta, float velocity) {
    // Apply filter to wheel delta if it's non-zero
    if (wheel_delta != 0) {
        filter_wheel(wheel_delta);
    }
    
    // user is actively turning (wheel_delta is non-zero)
    if (wheel_delta) {
        // cancel any ongoing free scroll
        freeScrollActive = false;

        // send the real-time scroll using TinyUSB
        scroll(wheel_delta);

        // capture momentum for possible fling
        freeMomentum = wheel_delta;
        freeLastTime = millis();
    }
    else {
        // no detent event right now
        unsigned long now = millis();

        // if we just stopped turning and were fast enough, kick off free scroll
        if (!freeScrollActive
            && fabsf(velocity) > FREE_THRESH
            && (now - freeLastTime) < 200)  // within 200 ms of last tick
        {
            freeScrollActive = true;
            freeLastTime = now; // Reset time for first free scroll interval
        }

        // Handle active free scroll
        if (freeScrollActive && (now - freeLastTime) >= FREE_INTERVAL) {
            freeMomentum *= FREE_FRICTION;
            int8_t fling = (int8_t)roundf(freeMomentum);

            if (abs(fling) < 1) {
                freeScrollActive = false; // Momentum died out
            } else {
                // Send scroll using TinyUSB
                scroll(fling);
                freeLastTime = now;
            }
        }
        // --- Alternative friction model (from original commented code) ---
        // if (freeScrollActive && (now - freeLastTime) >= FREE_INTERVAL) {
        //   // subtract a bit of momentum each step
        //   if (freeMomentum > 0) {
        //     freeMomentum = max(0.0f, freeMomentum - FREE_DECEL);
        //   } else {
        //     freeMomentum = min(0.0f, freeMomentum + FREE_DECEL);
        //   }
        //   // round for HID
        //   int8_t fling = (int8_t)roundf(freeMomentum);
        //   if (abs(fling) < 1) {
        //     freeScrollActive = false;  // coast has stopped
        //   } else {
        //     // Send scroll using TinyUSB instead of Mouse.move
        //     scroll(fling);
        //     freeLastTime = now;
        //   }
        // }
    }
} 

bool ScrollHandler::isFreeScrolling() const {
    return freeScrollActive;
}

//--------------------------------------------------------------------+
// Low pass filter Functions
//--------------------------------------------------------------------+

butterworth_coeffs_t ScrollHandler::butterworth_lowpass(float cutoff_frequency, float sampling_frequency) {
    butterworth_coeffs_t coe;

    float omega = 2.0 * PI * cutoff_frequency / sampling_frequency;
    float s = sin(omega);
    float t = tan(omega / 2.0);
    float alpha = s / (2.0 * t);

    coe.b0 = 1.0 / (1.0 + 2.0 * alpha + 2.0 * alpha * alpha);
    coe.b1 = 2.0 * coe.b0;
    coe.b2 = coe.b0;
    coe.a1 = 2.0 * (alpha * alpha - 1.0) * coe.b0;
    coe.a2 = (1.0 - 2.0 * alpha + 2.0 * alpha * alpha) * coe.b0;

    return coe;
}

float ScrollHandler::butterworth_filter(float data, butterworth_coeffs_t *coeffs, float *filtered, float *prev1, float *prev2) {
    float output = coeffs->b0 * data + coeffs->b1 * (*prev1) + coeffs->b2 * (*prev2) - 
                   coeffs->a1 * (*filtered) - coeffs->a2 * (*prev1);
    *prev2 = *prev1;
    *prev1 = data;
    *filtered = output;
    return output;
}

void ScrollHandler::filter_wheel(int8_t &wheel_delta) {
    float filtered_value = butterworth_filter(
        static_cast<float>(wheel_delta), 
        &wheel_coeffs, 
        &filtered_wheel, 
        &prev1_wheel, 
        &prev2_wheel
    );
    
    // Apply the filtered value back to the wheel_delta
    // Only if the filtered value is significant
    if (fabsf(filtered_value) >= 1.0f) {
        wheel_delta = static_cast<int8_t>(roundf(filtered_value));
    }
} 