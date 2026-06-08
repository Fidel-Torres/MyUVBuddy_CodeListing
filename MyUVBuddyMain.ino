// EVENT-DRIVEN LOW-POWER BUILD
// MCU sleeps fully until button long-press triggers pairing mode.
// On first BLE connect, dashboard sends SETTIME=<unix> to sync wall clock.
// All stored readings use sleeptimer-based wall timestamps.
// =============================================================================
// MyUVBuddy — System Integration Main Firmware
// Hardware : Seeed Studio XIAO MG24 (EFR32MG24)
// Sensor   : Adafruit LTR390 UV / Ambient Light
// Stack    : Tools > Protocol Stack > BLE (Silabs)
// =============================================================================

#include <Wire.h>
#include <string.h>
#include <stdlib.h>
#include "Adafruit_LTR390.h"
#include "sl_sleeptimer.h"
#include "sl_power_manager.h"
#include "MotorButton.h"

// =============================================================================
// Configuration
// =============================================================================

#define RF_SW_PW_PIN PB5
#define RF_SW_PIN    PB4

static const uint8_t  DEVICE_NAME[]       = "MYUVBUDDY";
static const uint32_t SAMPLE_INTERVAL_MS  = 5000u;
static const float    SAMPLE_INTERVAL_S   = 5.0f;
static const uint8_t  SENSOR_RETRY_LIMIT  = 3u;

static const uint16_t BLE_CONN_MIN_INT    = 400u;
static const uint16_t BLE_CONN_MAX_INT    = 800u;
static const uint16_t BLE_CONN_LATENCY    = 0u;
static const uint16_t BLE_CONN_TIMEOUT    = 600u;

static const float   UVI_THRESHOLD        = 3.0f;
static const uint8_t THRESHOLD_DEBOUNCE   = 2u;

static const float DEFAULT_SED_ALERT_THRESHOLD = 2.0f;
static float       g_sed_alert_threshold       = DEFAULT_SED_ALERT_THRESHOLD;

static const uint32_t UV_SENSITIVITY      = 2300u;
static const float    ALS_LUX_FACTOR      = 0.0083f;
static const float    SED_DOSE_RATE_FACTOR = 0.025f;

static const uint32_t SIG_UVI_HIGH        = (1u << 0);
static const uint32_t SIG_UVI_OK          = (1u << 1);
static const uint32_t SIG_SED_HIGH        = (1u << 2);

#define READ_BACKLOG_SIZE        300
#define BACKLOG_SEND_SPACING_MS  50

#define LOG_UVI_DELTA        0.2f
#define LOG_LUX_DELTA        10.0f
#define LOG_UV_RAW_DELTA     10u
#define LOG_SED_DELTA        0.005f
#define LOG_HEARTBEAT_MS     60000u

// =============================================================================
// UUIDs
// =============================================================================

const uuid_128 SPP_SERVICE_UUID = {
  .data = {
    0x07, 0xb9, 0xf9, 0xd7,
    0x50, 0xa4,
    0x20, 0x89,
    0x77, 0x40,
    0xcb, 0xfd,
    0x2c, 0xc1, 0x80, 0x48
  }
};

const uuid_128 SPP_DATA_CHAR_UUID = {
  .data = {
    0xd6, 0x58, 0xd6, 0x21,
    0xbc, 0x55,
    0x81, 0x9f,
    0x42, 0x44,
    0x71, 0x6d,
    0xc4, 0x6e, 0xc2, 0xfe
  }
};

// =============================================================================
// BLE state machine
// =============================================================================

typedef enum {
  STATE_DORMANT = 0,   // sleeping, not advertising — waits for button long-press
  STATE_DISCONNECTED,
  STATE_ADVERTISING,
  STATE_CONNECTED,
  STATE_SPP_MODE
} spp_state_t;

static volatile spp_state_t g_ble_state      = STATE_DORMANT;
static uint8_t              g_conn_handle     = SL_BT_INVALID_CONNECTION_HANDLE;
static uint8_t              g_max_packet_size = 20u;

static uint16_t g_gattdb_session;
static uint16_t g_ga_service_handle;
static uint16_t g_device_name_char_handle;
static uint16_t g_spp_service_handle;
static uint16_t g_spp_data_char_handle;

// =============================================================================
// Wall clock (sleeptimer-based, survives sleep)
// =============================================================================

static bool     g_time_synced      = false;
static uint32_t g_time_base_unix   = 0;
static uint32_t g_time_base_ticks  = 0;

static void apply_wall_time(uint32_t unix_ts)
{
  g_time_base_unix  = unix_ts;
  g_time_base_ticks = sl_sleeptimer_get_tick_count();
  g_time_synced     = true;
  Serial.print("[TIME] Synced to unix: ");
  Serial.println(unix_ts);
}

static uint32_t get_unix_time()
{
  if (!g_time_synced) return 0;
  uint32_t delta_ticks = sl_sleeptimer_get_tick_count() - g_time_base_ticks;
  uint32_t delta_s     = sl_sleeptimer_tick_to_ms(delta_ticks) / 1000u;
  return g_time_base_unix + delta_s;
}

// =============================================================================
// Sleep-timer and wakeup flag
// =============================================================================

static sl_sleeptimer_timer_handle_t g_sample_timer;
static volatile bool g_sensor_read_pending = false;

// =============================================================================
// Instant UVI threshold state
// =============================================================================

static bool    g_above_threshold     = false;
static uint8_t g_threshold_hit_count = 0u;

// =============================================================================
// Cumulative SED dose state
// =============================================================================

static float g_cumulative_J   = 0.0f;
static float g_cumulative_sed = 0.0f;
static bool  g_sed_alerted    = false;

// =============================================================================
// Sensor
// =============================================================================

static Adafruit_LTR390 g_ltr;
static bool            g_sensor_ok = false;

// =============================================================================
// Backlog — stores unix timestamp per reading
// =============================================================================

typedef struct {
  uint32_t seq;
  uint32_t unix_ts;   // wall time at capture (0 = unsynced)
  bool     synced;    // was clock synced when this was captured?
  float    uvi;
  float    lux;
  uint32_t uv_raw;
  float    sed;
} StoredReading;

static StoredReading g_backlog[READ_BACKLOG_SIZE];

static uint16_t g_backlog_head  = 0;
static uint16_t g_backlog_tail  = 0;
static uint16_t g_backlog_count = 0;

static uint32_t      g_sample_seq          = 0;
static unsigned long g_last_backlog_send_ms = 0;

static bool          g_have_last_stored  = false;
static float         g_last_stored_uvi    = 0.0f;
static float         g_last_stored_lux    = 0.0f;
static uint32_t      g_last_stored_uv_raw = 0;
static float         g_last_stored_sed    = 0.0f;
static unsigned long g_last_stored_ms     = 0;

// =============================================================================
// Forward declarations
// =============================================================================

static void initializeSystem();
static void initSerial();
static void initMotorButton();
static void initStatusLED();
static void initAntennaSwitch();
static void initSensor();

static void serviceRuntimeTasks();
static bool systemCanSleep();
static void enterIdleSleep();
static void handleButtonBleDisconnectRequest();
static void handleButtonPairingRequest();
static void handleSensorSampleEvent();
static void processUvSample(uint32_t uvs, uint32_t als);
static void handleSensorReadFailure();
static void logSensorReading(float uvi, float lux, uint32_t uvs);

static void  ble_init_gatt_db();
static void  ble_start_advertising();
static bool  ble_send_chunked(const uint8_t* data, size_t len);

static void  sample_timer_callback(sl_sleeptimer_timer_handle_t* handle, void* data);
static void  start_sample_timer();

static bool  sensor_read(uint32_t& out_uvs, uint32_t& out_als);
static float uvs_to_uvi(uint32_t uvs);
static float als_to_lux(uint32_t als);

static void  accumulate_sed(float uvi);
static void  check_uvi_threshold(float uvi);
static void  check_sed_threshold();
static void  reset_session();
static void  apply_sed_threshold(float threshold);
static void  handle_ble_command(const uint8_t* data, uint16_t len);

static bool  should_store_reading(float uvi, float lux, uint32_t uv_raw, float sed);
static void  store_reading(float uvi, float lux, uint32_t uv_raw, float sed);
static void  clear_backlog();
static void  send_pending_readings();
static void  service_delay(uint32_t ms);

// =============================================================================
// Arduino setup and loop
// =============================================================================

void setup()
{
  initializeSystem();
}

void loop()
{
  serviceRuntimeTasks();
  handleSensorSampleEvent();
  enterIdleSleep();
}

// =============================================================================
// System initialization
// Sensor timer and BLE advertising are NOT started here.
// They start only after the user long-presses to enter pairing mode.
// =============================================================================

static void initializeSystem()
{
  initSerial();
  initMotorButton();
  initStatusLED();
  initAntennaSwitch();
  initSensor();

  Serial.println("[SYSTEM] Dormant — long-press button to start pairing");
  Serial.println("[POWER] Event-driven sleep enabled");
}

static void initSerial()
{
  Serial.begin(115200);
  delay(1500);
  Serial.println("[UV BUDDY] Starting...");
}

static void initMotorButton()
{
  motorSetup();
}

static void initStatusLED()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
}

static void initAntennaSwitch()
{
  pinMode(RF_SW_PW_PIN, OUTPUT);
  digitalWrite(RF_SW_PW_PIN, HIGH);
  delay(100);
  pinMode(RF_SW_PIN, OUTPUT);
  digitalWrite(RF_SW_PIN, LOW);
}

static void initSensor()
{
  Wire.begin();
  g_sensor_ok = g_ltr.begin();

  if (!g_sensor_ok) {
    Serial.println("[SENSOR] LTR390 not found — check wiring!");
    return;
  }

  g_ltr.setGain(LTR390_GAIN_18);
  g_ltr.setResolution(LTR390_RESOLUTION_20BIT);

  Serial.print("[SENSOR] LTR390 ready — UVI threshold: ");
  Serial.print(UVI_THRESHOLD, 1);
  Serial.print("  SED threshold: ");
  Serial.println(g_sed_alert_threshold, 1);
}

// =============================================================================
// Runtime service loop
// =============================================================================

static void serviceRuntimeTasks()
{
  motorLoop();
  handleButtonPairingRequest();
  handleButtonBleDisconnectRequest();
  send_pending_readings();
}

// =============================================================================
// Pairing gate
// The MCU stays in STATE_DORMANT until handleButtonPairingRequest() fires.
// On first trigger: start advertising and start the sample timer.
//
// FIX #2: ble_init_gatt_db() is NOT called here.
// The GATT DB is built once inside sl_bt_evt_system_boot_id at stack boot.
// Calling ble_init_gatt_db() a second time would invoke sl_bt_gattdb_new_session()
// on an already-committed database, causing undefined behavior or a crash.
// =============================================================================

static void handleButtonPairingRequest()
{
  if (!motorTakePairingRequest()) return;

  if (g_ble_state != STATE_DORMANT) {
    // Already active — ignore repeated long-press
    return;
  }

  Serial.println("[SYSTEM] Pairing requested — starting BLE advertising + sensor timer");

  // GATT DB was already initialized in sl_bt_evt_system_boot_id — do not call again.
  ble_start_advertising();
  start_sample_timer();

  g_ble_state = STATE_ADVERTISING;
}

// =============================================================================
// Sleep control
// =============================================================================

static bool systemCanSleep()
{
  if (g_sensor_read_pending)                                       return false;
  if (motorHasPendingWork())                                       return false;
  if (g_ble_state == STATE_SPP_MODE && g_backlog_count > 0)       return false;
  return true;
}

static void enterIdleSleep()
{
  if (!systemCanSleep()) return;
  sl_power_manager_sleep();
}

static void handleButtonBleDisconnectRequest()
{
  if (!motorTakeBleDisconnectRequest()) return;

  if (g_conn_handle != SL_BT_INVALID_CONNECTION_HANDLE) {
    Serial.println("[BLE] Button requested disconnect");
    sl_bt_connection_close(g_conn_handle);
  } else {
    Serial.println("[BLE] Button requested disconnect, but no active connection");
  }
}

// =============================================================================
// Sensor sample event
// =============================================================================

static void handleSensorSampleEvent()
{
  if (!g_sensor_read_pending) return;
  g_sensor_read_pending = false;

  uint32_t uvs = 0;
  uint32_t als = 0;

  if (!sensor_read(uvs, als)) {
    handleSensorReadFailure();
    return;
  }

  processUvSample(uvs, als);
}

static void handleSensorReadFailure()
{
  Serial.println("[SENSOR] Read failed");
  if (g_ble_state == STATE_SPP_MODE) {
    const char* err = "ERR=SENSOR\n";
    ble_send_chunked((const uint8_t*)err, strlen(err));
  }
}

static void processUvSample(uint32_t uvs, uint32_t als)
{
  float uvi = uvs_to_uvi(uvs);
  float lux = als_to_lux(als);

  accumulate_sed(uvi);
  motorSetSedPercent((g_cumulative_sed / g_sed_alert_threshold) * 100.0f);

  if (should_store_reading(uvi, lux, uvs, g_cumulative_sed)) {
    store_reading(uvi, lux, uvs, g_cumulative_sed);
  } else {
    Serial.println("[LOG] Reading skipped - no significant change");
  }

  logSensorReading(uvi, lux, uvs);
  check_uvi_threshold(uvi);
  check_sed_threshold();
  send_pending_readings();
}

static void logSensorReading(float uvi, float lux, uint32_t uvs)
{
  Serial.print("[DATA] UVI=");
  Serial.print(uvi, 1);
  Serial.print(" LUX=");
  Serial.print(lux, 1);
  Serial.print(" UV=");
  Serial.print(uvs);
  Serial.print(" SED=");
  Serial.println(g_cumulative_sed, 3);
}

// =============================================================================
// Service delay
// =============================================================================

static void service_delay(uint32_t ms)
{
  unsigned long start = millis();
  while (millis() - start < ms) {
    motorLoop();
    delay(1);
  }
}

// =============================================================================
// Backlog filter
// =============================================================================

static bool should_store_reading(float uvi, float lux, uint32_t uv_raw, float sed)
{
  if (g_ble_state == STATE_SPP_MODE) return true;
  if (!g_have_last_stored)           return true;

  unsigned long now = millis();
  if (now - g_last_stored_ms >= LOG_HEARTBEAT_MS) return true;

  float duvi = uvi - g_last_stored_uvi; if (duvi < 0) duvi = -duvi;
  float dlux = lux - g_last_stored_lux; if (dlux < 0) dlux = -dlux;
  uint32_t duv = (uv_raw > g_last_stored_uv_raw)
                 ? (uv_raw - g_last_stored_uv_raw)
                 : (g_last_stored_uv_raw - uv_raw);
  float dsed = sed - g_last_stored_sed; if (dsed < 0) dsed = -dsed;

  if (duvi   >= LOG_UVI_DELTA)    return true;
  if (dlux   >= LOG_LUX_DELTA)    return true;
  if (duv    >= LOG_UV_RAW_DELTA) return true;
  if (dsed   >= LOG_SED_DELTA)    return true;

  return false;
}

// =============================================================================
// Backlog storage — timestamps use sleeptimer wall clock
// =============================================================================

static void store_reading(float uvi, float lux, uint32_t uv_raw, float sed)
{
  if (g_backlog_count == READ_BACKLOG_SIZE) {
    g_backlog_tail = (g_backlog_tail + 1) % READ_BACKLOG_SIZE;
    g_backlog_count--;
    Serial.println("[LOG] Backlog full - oldest reading dropped");
  }

  StoredReading& r = g_backlog[g_backlog_head];

  r.seq     = g_sample_seq++;
  r.unix_ts = get_unix_time();
  r.synced  = g_time_synced;
  r.uvi     = uvi;
  r.lux     = lux;
  r.uv_raw  = uv_raw;
  r.sed     = sed;

  g_backlog_head = (g_backlog_head + 1) % READ_BACKLOG_SIZE;
  g_backlog_count++;

  Serial.print("[LOG] Stored seq=");
  Serial.print(r.seq);
  Serial.print(" unix=");
  Serial.print(r.unix_ts);
  Serial.print(" synced=");
  Serial.println(r.synced ? "yes" : "no");

  g_have_last_stored   = true;
  g_last_stored_uvi    = uvi;
  g_last_stored_lux    = lux;
  g_last_stored_uv_raw = uv_raw;
  g_last_stored_sed    = sed;
  g_last_stored_ms     = millis();
}

static void clear_backlog()
{
  g_backlog_head       = 0;
  g_backlog_tail       = 0;
  g_backlog_count      = 0;
  g_sample_seq         = 0;
  g_last_backlog_send_ms = 0;
  g_have_last_stored   = false;
  g_last_stored_uvi    = 0.0f;
  g_last_stored_lux    = 0.0f;
  g_last_stored_uv_raw = 0;
  g_last_stored_sed    = 0.0f;
  g_last_stored_ms     = 0;
  Serial.println("[LOG] Backlog cleared");
}

// =============================================================================
// Send pending readings
// Packet format: SEQ=n T=unix UVI=x.x LUX=x.x UV=n SED=x.xxx SYNCED=1\n
// =============================================================================

static void send_pending_readings()
{
  if (g_ble_state != STATE_SPP_MODE) return;
  if (g_backlog_count == 0)          return;

  unsigned long now = millis();
  if (now - g_last_backlog_send_ms < BACKLOG_SEND_SPACING_MS) return;

  StoredReading& r = g_backlog[g_backlog_tail];

  char msg[120];
  int len = snprintf(msg, sizeof(msg),
    "SEQ=%lu T=%lu UVI=%.1f LUX=%.1f UV=%lu SED=%.3f SYNCED=%d\n",
    r.seq,
    r.unix_ts,
    r.uvi,
    r.lux,
    r.uv_raw,
    r.sed,
    r.synced ? 1 : 0
  );

  if (len <= 0) return;

  if (ble_send_chunked((const uint8_t*)msg, (size_t)len)) {
    g_backlog_tail = (g_backlog_tail + 1) % READ_BACKLOG_SIZE;
    g_backlog_count--;
    g_last_backlog_send_ms = now;

    Serial.print("[BLE LOG] Sent seq=");
    Serial.print(r.seq);
    Serial.print(" remaining=");
    Serial.println(g_backlog_count);
  }
}

// =============================================================================
// SED accumulation
// =============================================================================

static void accumulate_sed(float uvi)
{
  if (uvi < 0.0f) return;
  g_cumulative_J   += uvi * SED_DOSE_RATE_FACTOR * SAMPLE_INTERVAL_S;
  g_cumulative_sed  = g_cumulative_J / 100.0f;
}

static void check_sed_threshold()
{
  if (!g_sed_alerted && g_cumulative_sed >= g_sed_alert_threshold) {
    g_sed_alerted = true;
    sl_bt_external_signal(SIG_SED_HIGH);
  }
}

static void check_uvi_threshold(float uvi)
{
  if (uvi >= UVI_THRESHOLD) {
    if (!g_above_threshold) {
      g_threshold_hit_count++;
      if (g_threshold_hit_count >= THRESHOLD_DEBOUNCE) {
        g_above_threshold = true;
        sl_bt_external_signal(SIG_UVI_HIGH);
      }
    }
  } else {
    g_threshold_hit_count = 0u;
    if (g_above_threshold) {
      g_above_threshold = false;
      sl_bt_external_signal(SIG_UVI_OK);
    }
  }
}

// =============================================================================
// Session reset
// =============================================================================

static void reset_session()
{
  g_cumulative_J        = 0.0f;
  g_cumulative_sed      = 0.0f;
  g_sed_alerted         = false;
  g_above_threshold     = false;
  g_threshold_hit_count = 0u;
  motorSetSedPercent(0.0f);
  clear_backlog();
  Serial.println("[SESSION] Reset — cumulative SED cleared");
}

// =============================================================================
// Dose profile command
// =============================================================================

static void apply_sed_threshold(float threshold)
{
  if (threshold < 0.5f || threshold > 10.0f) {
    const char* err = "ERR=BAD_SED\n";
    ble_send_chunked((const uint8_t*)err, strlen(err));
    return;
  }

  g_sed_alert_threshold = threshold;
  motorSetSedPercent((g_cumulative_sed / g_sed_alert_threshold) * 100.0f);
  g_sed_alerted = false;
  check_sed_threshold();

  Serial.print("[CONFIG] SED threshold: ");
  Serial.println(g_sed_alert_threshold, 1);

  char ack[32];
  snprintf(ack, sizeof(ack), "ACK=SED %.1f\n", g_sed_alert_threshold);
  ble_send_chunked((const uint8_t*)ack, strlen(ack));
}

// =============================================================================
// BLE command handler
// Supported commands: RESET | SETSED=x.x | SETTIME=unix
// =============================================================================

static void handle_ble_command(const uint8_t* data, uint16_t len)
{
  char cmd[64];
  if (len >= sizeof(cmd)) len = sizeof(cmd) - 1;
  memcpy(cmd, data, len);
  cmd[len] = '\0';

  if (strncmp(cmd, "RESET", 5) == 0) {
    reset_session();
    const char* ack = "ACK=RESET\n";
    ble_send_chunked((const uint8_t*)ack, strlen(ack));
    return;
  }

  if (strncmp(cmd, "SETSED=", 7) == 0) {
    apply_sed_threshold(atof(cmd + 7));
    return;
  }

  if (strncmp(cmd, "SETTIME=", 8) == 0) {
    uint32_t unix_ts = (uint32_t)atol(cmd + 8);
    apply_wall_time(unix_ts);
    const char* ack = "ACK=TIME\n";
    ble_send_chunked((const uint8_t*)ack, strlen(ack));
    return;
  }
}

// =============================================================================
// Conversions
// =============================================================================

static float uvs_to_uvi(uint32_t uvs) { return (float)uvs / (float)UV_SENSITIVITY; }
static float als_to_lux(uint32_t als) { return (float)als * ALS_LUX_FACTOR; }

// =============================================================================
// Sensor read with retry
// =============================================================================

static bool sensor_read(uint32_t& out_uvs, uint32_t& out_als)
{
  if (!g_sensor_ok) return false;

  for (uint8_t attempt = 0; attempt < SENSOR_RETRY_LIMIT; attempt++) {
    g_ltr.setMode(LTR390_MODE_ALS);
    service_delay(420);
    if (!g_ltr.newDataAvailable()) service_delay(80);
    out_als = g_ltr.readALS();

    g_ltr.setMode(LTR390_MODE_UVS);
    service_delay(420);
    if (!g_ltr.newDataAvailable()) service_delay(80);
    out_uvs = g_ltr.readUVS();

    if (out_als != 0xFFFFFFFF && out_uvs != 0xFFFFFFFF) return true;

    Serial.print("[SENSOR] Retry ");
    Serial.println(attempt + 1);
  }

  return false;
}

// =============================================================================
// Sleep-timer ISR
// =============================================================================

static void sample_timer_callback(sl_sleeptimer_timer_handle_t* /*handle*/,
                                  void* /*data*/)
{
  g_sensor_read_pending = true;
}

static void start_sample_timer()
{
  sl_sleeptimer_stop_timer(&g_sample_timer);

  uint32_t ticks = sl_sleeptimer_ms_to_tick(SAMPLE_INTERVAL_MS);

  sl_status_t sc = sl_sleeptimer_start_periodic_timer(
    &g_sample_timer,
    ticks,
    sample_timer_callback,
    nullptr,
    0,
    SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG
  );

  if (sc != SL_STATUS_OK) {
    Serial.print("[TIMER] Failed: 0x");
    Serial.println(sc, HEX);
  } else {
    Serial.println("[TIMER] Periodic sample timer started (5 s)");
  }
}

// =============================================================================
// BLE send chunked
// =============================================================================

static bool ble_send_chunked(const uint8_t* data, size_t len)
{
  if (g_ble_state != STATE_SPP_MODE) return false;

  size_t offset = 0;
  while (offset < len) {
    size_t chunk = min((size_t)g_max_packet_size, len - offset);

    sl_status_t sc = sl_bt_gatt_server_send_notification(
      g_conn_handle,
      g_spp_data_char_handle,
      (uint16_t)chunk,
      data + offset
    );

    if (sc != SL_STATUS_OK) {
      Serial.print("[BLE] Notification error: 0x");
      Serial.println(sc, HEX);
      return false;
    }

    offset += chunk;
    if (offset < len) delay(2);
  }

  return true;
}

// =============================================================================
// BLE Stack Event Handler
// =============================================================================

void sl_bt_on_event(sl_bt_msg_t* evt)
{
  switch (SL_BT_MSG_ID(evt->header)) {

    case sl_bt_evt_system_boot_id:
      Serial.println("[BLE] Stack booted — waiting for button long-press to start");
      // FIX #2: GATT DB is built HERE, once, at stack boot.
      // handleButtonPairingRequest() must NOT call ble_init_gatt_db() again.
      ble_init_gatt_db();
      // Do NOT call ble_start_advertising() here — wait for button long-press.
      break;

    case sl_bt_evt_connection_opened_id:
      Serial.println("[BLE] Connected");
      g_ble_state   = STATE_CONNECTED;
      g_conn_handle = evt->data.evt_connection_opened.connection;

      sl_bt_connection_set_parameters(
        g_conn_handle,
        BLE_CONN_MIN_INT, BLE_CONN_MAX_INT,
        BLE_CONN_LATENCY, BLE_CONN_TIMEOUT,
        0, 0xFFFF
      );
      break;

    case sl_bt_evt_connection_closed_id:
      Serial.print("[BLE] Disconnected, reason: 0x");
      Serial.println(evt->data.evt_connection_closed.reason, HEX);

      stopHaptic();
      g_ble_state   = STATE_ADVERTISING;
      g_conn_handle = SL_BT_INVALID_CONNECTION_HANDLE;
      digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
      ble_start_advertising();
      break;

    case sl_bt_evt_gatt_mtu_exchanged_id:
      g_max_packet_size =
        (uint8_t)(evt->data.evt_gatt_mtu_exchanged.mtu - 3u);
      Serial.print("[BLE] MTU: ");
      Serial.println(g_max_packet_size + 3);
      break;

    case sl_bt_evt_gatt_server_attribute_value_id:
    {
      auto& av = evt->data.evt_gatt_server_attribute_value;
      if (av.attribute == g_spp_data_char_handle) {
        handle_ble_command(av.value.data, av.value.len);
      }
    }
    break;

    case sl_bt_evt_gatt_server_characteristic_status_id:
    {
      auto& cs = evt->data.evt_gatt_server_characteristic_status;
      if (cs.characteristic == g_spp_data_char_handle) {
        if (cs.client_config_flags == sl_bt_gatt_server_notification) {
          g_ble_state = STATE_SPP_MODE;
          digitalWrite(LED_BUILTIN, LED_BUILTIN_ACTIVE);
          Serial.println("[BLE] Notifications enabled — SPP mode active");

          // Request time sync from dashboard immediately after connecting.
          const char* req = "CMD=REQTIME\n";
          ble_send_chunked((const uint8_t*)req, strlen(req));

          notifyBLEConnected();

          Serial.print("[BLE] Backlog pending = ");
          Serial.println(g_backlog_count);
        } else {
          g_ble_state = STATE_CONNECTED;
          digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
          Serial.println("[BLE] Notifications disabled");
        }
      }
    }
    break;

    case sl_bt_evt_system_external_signal_id:
    {
      uint32_t signals = evt->data.evt_system_external_signal.extsignals;

      if (signals & SIG_UVI_HIGH) {
        Serial.println("[ALERT] UVI above threshold");
        const char* alert = "ALERT=UVI_HIGH\n";
        ble_send_chunked((const uint8_t*)alert, strlen(alert));
        digitalWrite(LED_BUILTIN, LED_BUILTIN_ACTIVE);
        triggerUVAlert();
      }

      if (signals & SIG_UVI_OK) {
        Serial.println("[ALERT] UVI back below threshold");
        const char* alert = "ALERT=UVI_OK\n";
        ble_send_chunked((const uint8_t*)alert, strlen(alert));
        digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
      }

      if (signals & SIG_SED_HIGH) {
        Serial.print("[ALERT] SED threshold reached: ");
        Serial.println(g_cumulative_sed, 3);
        const char* alert = "ALERT=SED_HIGH\n";
        ble_send_chunked((const uint8_t*)alert, strlen(alert));
        triggerUVAlert();
      }
    }
    break;

    case sl_bt_evt_connection_parameters_id:
    {
      auto& p = evt->data.evt_connection_parameters;
      Serial.print("[BLE] Conn params — interval: ");
      Serial.print(p.interval);
      Serial.print(" latency: ");
      Serial.print(p.latency);
      Serial.print(" timeout: ");
      Serial.println(p.timeout);
    }
    break;

    default:
      break;
  }
}

// =============================================================================
// Advertising
// =============================================================================

static void ble_start_advertising()
{
  static uint8_t adv_handle  = 0xFF;
  static bool    initialized = false;

  if (!initialized) {
    sl_bt_advertiser_create_set(&adv_handle);
    sl_bt_advertiser_set_timing(adv_handle, 400, 400, 0, 0);
    initialized = true;
  }

  sl_bt_legacy_advertiser_generate_data(
    adv_handle,
    sl_bt_advertiser_general_discoverable
  );

  sl_bt_legacy_advertiser_start(
    adv_handle,
    sl_bt_advertiser_connectable_scannable
  );

  Serial.println("[BLE] Advertising...");
}

// =============================================================================
// GATT Database
// Called once at sl_bt_evt_system_boot_id. Must not be called again.
// =============================================================================

static void ble_init_gatt_db()
{
  sl_bt_gattdb_new_session(&g_gattdb_session);

  const uint8_t ga_uuid[] = { 0x00, 0x18 };

  sl_bt_gattdb_add_service(
    g_gattdb_session,
    sl_bt_gattdb_primary_service,
    SL_BT_GATTDB_ADVERTISED_SERVICE,
    sizeof(ga_uuid),
    ga_uuid,
    &g_ga_service_handle
  );

  const sl_bt_uuid_16_t dev_name_uuid = { .data = { 0x00, 0x2A } };

  sl_bt_gattdb_add_uuid16_characteristic(
    g_gattdb_session,
    g_ga_service_handle,
    SL_BT_GATTDB_CHARACTERISTIC_READ,
    0x00, 0x00,
    dev_name_uuid,
    sl_bt_gattdb_fixed_length_value,
    sizeof(DEVICE_NAME) - 1,
    sizeof(DEVICE_NAME) - 1,
    DEVICE_NAME,
    &g_device_name_char_handle
  );

  sl_bt_gattdb_start_service(g_gattdb_session, g_ga_service_handle);

  sl_bt_gattdb_add_service(
    g_gattdb_session,
    sl_bt_gattdb_primary_service,
    SL_BT_GATTDB_ADVERTISED_SERVICE,
    sizeof(SPP_SERVICE_UUID),
    SPP_SERVICE_UUID.data,
    &g_spp_service_handle
  );

  uint8_t init_val = 0;

  sl_bt_gattdb_add_uuid128_characteristic(
    g_gattdb_session,
    g_spp_service_handle,
    SL_BT_GATTDB_CHARACTERISTIC_WRITE_NO_RESPONSE |
    SL_BT_GATTDB_CHARACTERISTIC_NOTIFY,
    0x00, 0x00,
    SPP_DATA_CHAR_UUID,
    sl_bt_gattdb_fixed_length_value,
    250,
    sizeof(init_val),
    &init_val,
    &g_spp_data_char_handle
  );

  sl_bt_gattdb_start_service(g_gattdb_session, g_spp_service_handle);
  sl_bt_gattdb_commit(g_gattdb_session);

  Serial.println("[BLE] GATT DB initialized");
}
