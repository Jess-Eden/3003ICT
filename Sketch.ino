
#include <ESP32Servo.h>

// ── Stepper (A4988) Pins ─────────────────────────────────────
#define STEP_PIN   12   // A4988 STEP
#define DIR_PIN    13   // A4988 DIR

// ── Sensor / Input Pins ──────────────────────────────────────
#define PIR_PIN        14
#define DOOR_BTN_PIN    2   // reassigned – D12 used by stepper
#define ARM_BTN_PIN     4   // reassigned – D13 used by stepper
#define LDR_PIN        34
#define TRIG_PIN        5
#define ECHO_PIN       18
