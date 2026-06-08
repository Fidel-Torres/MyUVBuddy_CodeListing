#ifndef MOTOR_BUTTON_H
#define MOTOR_BUTTON_H

#include <Arduino.h>

// ─────────────────────────────────────────────
// DEBUG
// ─────────────────────────────────────────────
#define DEBUG_MODE 1

#if DEBUG_MODE
  #define ALERT_REPEAT_MS   10000
#else
  #define ALERT_REPEAT_MS   30000
#endif

// ─────────────────────────────────────────────
// PIN CONFIG
// ─────────────────────────────────────────────
#define MOTOR_PIN      D9
#define BUTTON_PIN     D10
#define DEBOUNCE_MS    50
#define LONG_PRESS_MS  2000
#define DOUBLE_TAP_MS  400
#define TRIPLE_TAP_MS  600
#define SNOOZE_MS      30000

// ─────────────────────────────────────────────
// PWM LEVELS
// ─────────────────────────────────────────────
#define PWM_OFF   0
#define PWM_LOW   150
#define PWM_MED   200
#define PWM_HIGH  230
#define PWM_MAX   255

// ─────────────────────────────────────────────
// TYPES
// ─────────────────────────────────────────────
typedef struct {
    uint8_t  duty;
    uint16_t on_ms;
    uint16_t off_ms;
    uint8_t  pulses;
} MotorPattern;

typedef enum {
    SYS_IDLE,
    SYS_ALERT,
    SYS_BLE_PAIRING,
    SYS_BLE_CONNECTED
} sys_state_t;

typedef enum {
    MOTOR_IDLE,
    MOTOR_PULSE_ON,
    MOTOR_PULSE_OFF
} motor_state_t;

// ─────────────────────────────────────────────
// API FUNCTIONS USED BY MyUVBuddyMain.ino
// ─────────────────────────────────────────────
void motorSetup();
void motorLoop();

// Returns true only when the module has immediate work already pending.
// Future button/haptic events are scheduled with a sleep timer, so the main
// firmware can still enter sleep until that timer or a GPIO interrupt wakes it.
bool motorHasPendingWork();

void triggerUVAlert();
void notifyBLEConnected();
void stopHaptic();

void motorSetSedPercent(float percent);
bool motorTakeBleDisconnectRequest();
bool motorTakePairingRequest();

#endif
