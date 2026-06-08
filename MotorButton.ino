#include "MotorButton.h"
#include "sl_sleeptimer.h"
#include "sl_power_manager.h"   // FIX #3: needed for EM1 requirement to prevent PWM freeze in EM2 sleep

// ─────────────────────────────────────────────
// EVENT-DRIVEN BUTTON/HAPTIC MODULE
// Button input wakes the MCU through a GPIO interrupt.
// Haptic timing, tap-window expiration, snooze timing, and alert repeat timing
// are scheduled with a one-shot sleep timer so the main loop does not need to
// stay awake just to poll timing.
// ─────────────────────────────────────────────

// ─────────────────────────────────────────────
// MOTOR PATTERNS
// ─────────────────────────────────────────────
static const MotorPattern PAT_UV_ALERT       = { PWM_MAX,  400, 300, 3 };
static const MotorPattern PAT_SNOOZE         = { PWM_MED,  150, 100, 2 };
static const MotorPattern PAT_DISMISS        = { PWM_LOW,  100,   0, 1 };
static const MotorPattern PAT_BLE_PAIRING    = { PWM_MED,  100, 150, 2 };
static const MotorPattern PAT_BLE_CONNECTED  = { PWM_HIGH, 600,   0, 1 };
static const MotorPattern PAT_BLE_CANCELLED  = { PWM_LOW,  100,   0, 1 };

// ─────────────────────────────────────────────
// STATE VARIABLES
// ─────────────────────────────────────────────
static volatile sys_state_t systemState = SYS_IDLE;

static volatile bool btnFalling = false;
static volatile bool btnRising  = false;

// Set by button ISR or internal sleep timer. When true, main should not sleep.
static volatile bool g_motorWorkPending = false;

static motor_state_t motorState = MOTOR_IDLE;
static MotorPattern activePattern;

static uint8_t currentPulse = 0;
static unsigned long motorLastTime = 0;
static unsigned long pressStart = 0;
static unsigned long lastAlertTime = 0;

static float g_sedPercent = 0.0f;
static bool g_bleConnected = false;
static bool bleNotifyPending = false;
static volatile bool g_bleDisconnectRequested = false;
static volatile bool g_blePairingRequested = false;

static uint8_t tapCount = 0;
static unsigned long firstTapTime = 0;
static unsigned long snoozeUntil = 0;

// FIX #3: tracks whether an EM1 requirement is currently held.
// Prevents double-add and double-release of the power manager requirement.
static bool g_em1_required = false;

// One-shot sleep timer used to wake the MCU for the next haptic/button event.
static sl_sleeptimer_timer_handle_t g_motor_service_timer;

// ─────────────────────────────────────────────
// FORWARD DECLARATIONS
// ─────────────────────────────────────────────
static void motorStart(const MotorPattern& p);
static void motorOff();
static void motorUpdate();
static void alertUpdate();
static void buttonISR();

static void handleSingleTap();
static void handleDoubleTap();
static void handleTripleTap();
static void handleLongPress();
static void playSedPercentPattern();

static void motorServiceTimerCallback(sl_sleeptimer_timer_handle_t* handle, void* data);
static void scheduleMotorServiceMs(uint32_t delay_ms);
static void scheduleNextMotorService();

#if DEBUG_MODE
static void debugUpdate();
static const char* getStateName(sys_state_t s);
#endif

// ─────────────────────────────────────────────
// SLEEP TIMER SERVICE
// ─────────────────────────────────────────────
static void motorServiceTimerCallback(sl_sleeptimer_timer_handle_t* /*handle*/,
                                      void* /*data*/)
{
    g_motorWorkPending = true;
}

static void scheduleMotorServiceMs(uint32_t delay_ms)
{
    if (delay_ms == 0) {
        g_motorWorkPending = true;
        return;
    }

    sl_sleeptimer_stop_timer(&g_motor_service_timer);

    uint32_t ticks = sl_sleeptimer_ms_to_tick(delay_ms);

    sl_sleeptimer_start_timer(
        &g_motor_service_timer,
        ticks,
        motorServiceTimerCallback,
        nullptr,
        0,
        SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG
    );
}

static void scheduleNextMotorService()
{
    unsigned long now = millis();
    bool scheduled = false;
    unsigned long nextDelay = 0xFFFFFFFFUL;

    // Next motor pulse edge.
    if (motorState == MOTOR_PULSE_ON) {
        unsigned long elapsed = now - motorLastTime;
        unsigned long remaining = (elapsed >= activePattern.on_ms)
                                  ? 1UL
                                  : (activePattern.on_ms - elapsed);
        nextDelay = min(nextDelay, remaining);
        scheduled = true;
    } else if (motorState == MOTOR_PULSE_OFF) {
        unsigned long elapsed = now - motorLastTime;
        unsigned long remaining = (elapsed >= activePattern.off_ms)
                                  ? 1UL
                                  : (activePattern.off_ms - elapsed);
        nextDelay = min(nextDelay, remaining);
        scheduled = true;
    }

    // FIX #4 (minor): use TRIPLE_TAP_MS as the tap window so only one wakeup
    // is needed instead of two (the earlier DOUBLE_TAP_MS + 5 wakeup was
    // redundant — scheduleNextMotorService already recalculates the remainder).
    if (tapCount > 0) {
        unsigned long elapsed = now - firstTapTime;
        unsigned long remaining = (elapsed > TRIPLE_TAP_MS)
                                ? 1UL
                                : (TRIPLE_TAP_MS - elapsed + 1UL);
        nextDelay = min(nextDelay, remaining);
        scheduled = true;
    }

    // Repeating alert or snooze expiration.
    if (systemState == SYS_ALERT && motorState == MOTOR_IDLE) {
        unsigned long alertDue = lastAlertTime + ALERT_REPEAT_MS;

        if (snoozeUntil != 0 && snoozeUntil > now) {
            alertDue = max(alertDue, snoozeUntil);
        }

        unsigned long remaining = (now >= alertDue) ? 1UL : (alertDue - now);
        nextDelay = min(nextDelay, remaining);
        scheduled = true;
    }

    if (scheduled) {
        if (nextDelay > 60000UL) {
            nextDelay = 60000UL;
        }

        scheduleMotorServiceMs((uint32_t)nextDelay);
    }
}

// ─────────────────────────────────────────────
// MOTOR CONTROL
// ─────────────────────────────────────────────

// FIX #3: motorOff releases the EM1 requirement so the MCU can enter EM2 sleep.
// The motor pin is guaranteed LOW before releasing, preventing the stuck-HIGH bug.
static void motorOff()
{
    analogWrite(MOTOR_PIN, PWM_OFF);
    motorState = MOTOR_IDLE;
    currentPulse = 0;

    // Release EM1 power requirement — motor is off, EM2 sleep is now safe.
    if (g_em1_required) {
        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
        g_em1_required = false;
    }
}

// FIX #3: motorStart acquires an EM1 requirement before enabling PWM.
// This prevents sl_power_manager_sleep() from entering EM2 while the PWM
// timer is running. EM2 halts HFRCO, which freezes the PWM peripheral and
// leaves the motor pin stuck HIGH until the next explicit write.
static void motorStart(const MotorPattern& p)
{
    // Block EM2 sleep for the duration of this haptic pattern.
    if (!g_em1_required) {
        sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
        g_em1_required = true;
    }

    activePattern = p;
    currentPulse = 0;
    motorLastTime = millis();
    motorState = MOTOR_PULSE_ON;
    analogWrite(MOTOR_PIN, p.duty);

    scheduleMotorServiceMs(p.on_ms);
}

static void motorUpdate()
{
    if (motorState == MOTOR_IDLE) {
        if (bleNotifyPending && systemState != SYS_ALERT) {
            bleNotifyPending = false;
            systemState = SYS_BLE_CONNECTED;
            motorStart(PAT_BLE_CONNECTED);
            Serial.println("[BLE] Connected (delayed notify)");
        }
        return;
    }

    unsigned long now = millis();
    unsigned long elapsed = now - motorLastTime;

    switch (motorState) {
        case MOTOR_PULSE_ON:
            if (elapsed >= activePattern.on_ms) {
                analogWrite(MOTOR_PIN, PWM_OFF);
                currentPulse++;

                if (currentPulse >= activePattern.pulses) {
                    // Pattern complete — release EM1 requirement via motorOff path.
                    motorState = MOTOR_IDLE;
                    if (g_em1_required) {
                        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
                        g_em1_required = false;
                    }
                } else {
                    motorState = MOTOR_PULSE_OFF;
                    motorLastTime = now;
                    scheduleMotorServiceMs(activePattern.off_ms);
                }
            }
            break;

        case MOTOR_PULSE_OFF:
            if (elapsed >= activePattern.off_ms) {
                analogWrite(MOTOR_PIN, activePattern.duty);
                motorState = MOTOR_PULSE_ON;
                motorLastTime = now;
                scheduleMotorServiceMs(activePattern.on_ms);
            }
            break;

        default:
            break;
    }
}

// ─────────────────────────────────────────────
// ALERT SYSTEM
// ─────────────────────────────────────────────
static void alertUpdate()
{
    if (systemState != SYS_ALERT) return;
    if (motorState != MOTOR_IDLE) return;

    unsigned long now = millis();

    // If user snoozed the alert, do not repeat until snooze expires.
    if (snoozeUntil != 0 && now < snoozeUntil) {
        scheduleMotorServiceMs((uint32_t)min(60000UL, snoozeUntil - now));
        return;
    }

    if (now - lastAlertTime >= ALERT_REPEAT_MS) {
        lastAlertTime = now;
        motorStart(PAT_UV_ALERT);
        Serial.println("[ALERT] UV alert repeated");
    } else {
        unsigned long remaining = ALERT_REPEAT_MS - (now - lastAlertTime);
        scheduleMotorServiceMs((uint32_t)min(60000UL, remaining));
    }
}

// ─────────────────────────────────────────────
// BUTTON ISR
// ISR does not classify the press. It only records the edge and wakes loop().
// ─────────────────────────────────────────────
static void buttonISR()
{
    if (digitalRead(BUTTON_PIN) == LOW) {
        btnFalling = true;
    } else {
        btnRising = true;
    }

    g_motorWorkPending = true;
}

// ─────────────────────────────────────────────
// BUTTON ACTIONS
// ─────────────────────────────────────────────
static void handleSingleTap()
{
    if (systemState == SYS_ALERT) {
        motorOff();

        // Snooze means pause alert repeats temporarily.
        snoozeUntil = millis() + SNOOZE_MS;
        lastAlertTime = millis();

        motorStart(PAT_SNOOZE);
        Serial.println("[BTN] Alert snoozed");
    } else {
        Serial.println("[BTN] Short tap - no active alert");
    }
}

static void handleDoubleTap()
{
    if (systemState == SYS_ALERT) {
        bleNotifyPending = false;
        motorOff();

        // Stop/acknowledge means leave alert state.
        if (g_bleConnected) {
            systemState = SYS_BLE_CONNECTED;
        } else {
            systemState = SYS_IDLE;
        }

        snoozeUntil = 0;
        lastAlertTime = millis();

        motorStart(PAT_DISMISS);
        Serial.println("[BTN] Alert acknowledged/stopped");
    } else {
        Serial.println("[BTN] Double tap - no active alert to stop");
    }
}

static void handleTripleTap()
{
    Serial.print("[BTN] Triple tap - SED status: ");
    Serial.print(g_sedPercent, 1);
    Serial.println("%");

    motorOff();
    playSedPercentPattern();
}

static void handleLongPress()
{
    if (g_bleConnected) {
        g_bleDisconnectRequested = true;
        motorOff();
        motorStart(PAT_BLE_CANCELLED);
        Serial.println("[BTN] BLE disconnect requested");
    } else {
        g_blePairingRequested = true;
        systemState = SYS_BLE_PAIRING;
        motorOff();
        motorStart(PAT_BLE_PAIRING);
        Serial.println("[BTN] BLE pairing mode");
    }
}

static void playSedPercentPattern()
{
    MotorPattern p;

    if (g_sedPercent < 25.0f) {
        p = { PWM_LOW, 120, 150, 1 };
        Serial.println("[SED] 0-25% dose");
    } else if (g_sedPercent < 50.0f) {
        p = { PWM_LOW, 120, 150, 2 };
        Serial.println("[SED] 25-50% dose");
    } else if (g_sedPercent < 80.0f) {
        p = { PWM_MED, 120, 150, 3 };
        Serial.println("[SED] 50-80% dose");
    } else if (g_sedPercent < 100.0f) {
        p = { PWM_HIGH, 120, 120, 4 };
        Serial.println("[SED] 80-100% dose");
    } else {
        p = PAT_UV_ALERT;
        Serial.println("[SED] 100%+ dose");
    }

    motorStart(p);
}

// ─────────────────────────────────────────────
// PUBLIC API
// ─────────────────────────────────────────────
void triggerUVAlert()
{
    bleNotifyPending = false;
    motorOff();

    systemState = SYS_ALERT;
    lastAlertTime = millis();

    // A new real alert cancels any previous snooze.
    snoozeUntil = 0;

    motorStart(PAT_UV_ALERT);
    Serial.println("[UV] Alert triggered");
}

void notifyBLEConnected()
{
    g_bleConnected = true;

    if (systemState == SYS_ALERT) {
        bleNotifyPending = true;
        g_motorWorkPending = true;
        Serial.println("[BLE] Connected pending (alert active)");
        return;
    }

    systemState = SYS_BLE_CONNECTED;
    motorStart(PAT_BLE_CONNECTED);
    Serial.println("[BLE] Connected");
}

void stopHaptic()
{
    bleNotifyPending = false;
    g_bleConnected = false;
    motorOff();

    if (systemState == SYS_BLE_CONNECTED || systemState == SYS_BLE_PAIRING) {
        systemState = SYS_IDLE;
    }

    Serial.println("[HAPTIC] Forced off");
}

void motorSetSedPercent(float percent)
{
    if (percent < 0.0f) {
        percent = 0.0f;
    }

    if (percent > 150.0f) {
        percent = 150.0f;
    }

    g_sedPercent = percent;
}

bool motorTakeBleDisconnectRequest()
{
    if (g_bleDisconnectRequested) {
        g_bleDisconnectRequested = false;
        return true;
    }

    return false;
}

bool motorTakePairingRequest()
{
    if (g_blePairingRequested) {
        g_blePairingRequested = false;
        return true;
    }
    return false;
}

bool motorHasPendingWork()
{
    return g_motorWorkPending || btnFalling || btnRising;
}

// ─────────────────────────────────────────────
// SETUP / LOOP
// ─────────────────────────────────────────────
void motorSetup()
{
    analogWriteResolution(8);
    pinMode(MOTOR_PIN, OUTPUT);

    // Use INPUT if your circuit has external 10k pull-up.
    // Use INPUT_PULLUP if the button is wired directly D10 → button → GND.
    pinMode(BUTTON_PIN, INPUT);

    motorOff();

    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, CHANGE);

    Serial.println("[MOTOR] Ready - interrupt button + sleep-timer haptic");
}

void motorLoop()
{
    g_motorWorkPending = false;

    motorUpdate();
    alertUpdate();

#if DEBUG_MODE
    debugUpdate();
#endif

    if (btnFalling) {
        btnFalling = false;
        pressStart = millis();
    }

    if (btnRising) {
        btnRising = false;

        static unsigned long lastRelease = 0;
        unsigned long now = millis();

        if (now - lastRelease >= DEBOUNCE_MS) {
            lastRelease = now;

            unsigned long duration = now - pressStart;
            bool isLong = (duration >= LONG_PRESS_MS);

            Serial.print("[BTN] ");
            Serial.print(isLong ? "Long " : "Short ");
            Serial.print(duration);
            Serial.println("ms");

            if (isLong) {
                tapCount = 0;
                handleLongPress();
            } else {
                if (tapCount == 0) {
                    firstTapTime = now;
                }

                tapCount++;

                // Cap at 3 so extra taps do not overflow behavior.
                if (tapCount > 3) {
                    tapCount = 3;
                }

                // FIX #4 (minor): schedule wakeup at the full TRIPLE_TAP_MS window
                // instead of DOUBLE_TAP_MS+5, eliminating an unnecessary early wakeup.
                // scheduleNextMotorService() will refine this every loop iteration.
                scheduleMotorServiceMs(TRIPLE_TAP_MS + 5);
            }
        }
    }

    if (tapCount > 0 && (millis() - firstTapTime > TRIPLE_TAP_MS)) {
        uint8_t finalTapCount = tapCount;
        tapCount = 0;

        if (finalTapCount >= 3) {
            handleTripleTap();
        } else if (finalTapCount == 2) {
            handleDoubleTap();
        } else {
            handleSingleTap();
        }
    }

    scheduleNextMotorService();
}

// ─────────────────────────────────────────────
// DEBUG
// ─────────────────────────────────────────────
#if DEBUG_MODE

static const char* getStateName(sys_state_t s)
{
    switch (s) {
        case SYS_IDLE: return "SYS_IDLE";
        case SYS_ALERT: return "SYS_ALERT";
        case SYS_BLE_PAIRING: return "SYS_BLE_PAIRING";
        case SYS_BLE_CONNECTED: return "SYS_BLE_CONNECTED";
        default: return "UNKNOWN";
    }
}

static void debugUpdate()
{
    if (!Serial.available()) return;

    char cmd = Serial.read();

    switch (cmd) {

        case 'A':
        case 'a':
            triggerUVAlert();
            break;

        case 'B':
        case 'b':
            notifyBLEConnected();
            break;

        case 'S':
        case 's':
            Serial.print("[DBG] systemState = ");
            Serial.println(getStateName(systemState));

            Serial.print("[DBG] SED percent = ");
            Serial.print(g_sedPercent, 1);
            Serial.println("%");
            break;

        case 'M':
        case 'm':
            Serial.print("[DBG] motorState = ");
            Serial.println(
                motorState == MOTOR_IDLE     ? "IDLE" :
                motorState == MOTOR_PULSE_ON ? "PULSE_ON" :
                                               "PULSE_OFF"
            );
            Serial.print("[DBG] EM1 held = ");
            Serial.println(g_em1_required ? "YES" : "NO");
            break;

        case 'P':
        case 'p':
        {
            float value = Serial.parseFloat();

            motorSetSedPercent(value);

            Serial.print("[DBG] SED set to ");
            Serial.print(value, 1);
            Serial.println("%");

            playSedPercentPattern();
            break;
        }

        case '?':
            Serial.println("Commands:");
            Serial.println("  A      = Trigger UV alert");
            Serial.println("  B      = Simulate BLE connected");
            Serial.println("  S      = Show system state");
            Serial.println("  M      = Show motor state + EM1 status");
            Serial.println("  P<num> = Test SED vibration");
            Serial.println("Examples:");
            Serial.println("  P10");
            Serial.println("  P50");
            Serial.println("  P75");
            Serial.println("  P100");
            Serial.println("  P125");
            break;

        default:
            break;
    }
}

#endif
