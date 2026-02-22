/*
 * Lakki Firmis.
 *
 * Author: Matti Vaittinen <mazziesaccount@gmail.com>
 *
 * No warranty. Use at own risk. May cause damage.
 *
 * This sketch implements a "navigation hat" :)
 * It is designed to match the Android BLE GATT client.
 *
 * The BLE connection has been written assisted by AI. Rest of
 * the stuff here is hand crafted.
 *
 * Copyright 2026 Matti Vaittinen <mazziesaccount@gmail.com>
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "ICM_20948.h"

#include "external_navigation_protocol.h"

#define LED_IND_LOOPS 1000;


static const int debug = 0;
#define ENABLE_BLE_DIRECTION_DEBUG 0

/*
 * Let's agree that the direction where the cap points at, is 0.
 * So, leftmost LED should indicate anything from behind user to directly left - Eg, 180 => 180 + 90
 */

#define DIR_BACK 180
#define DIR_LEFT (180 + 90)
#define DIR_RIGHT (180 - 90)

#define DIR_BACK_LEFT_LED (DIR_BACK + 45)
#define SECTOR_BACK_LEFT_LED 90

#define DIR_BACK_RIGHT_LED (DIR_BACK - 45)
#define SECTOR_BACK_RIGHT_LED 90

#define DIR_FRONT_LEFT_LED (DIR_LEFT + 35)
#define SECTOR_FRONT_LEFT_LED 70

#define DIR_FRONT_RIGHT_LED (DIR_RIGHT - 35)
#define SECTOR_FRONT_RIGHT_LED 70

#define DIR_FRONT_LED 0
#define SECTOR_FRONT_LED 40

struct mva_led {
  /* GPIO number*/
  int gpio_pin;
  /* Sector this LED points, relative to cap-dir*/
  uint16_t dir;
  uint16_t sector_width;
};

#define NUM_LEDS 5

static const struct mva_led g_led_arr[] =
{
  /* LEDs, left to right */
  {
    .gpio_pin = D2,
    .dir = DIR_BACK_LEFT_LED,
    .sector_width = SECTOR_BACK_LEFT_LED,
  }, {
    .gpio_pin = D3,
    .dir = DIR_FRONT_LEFT_LED,
    .sector_width = SECTOR_FRONT_LEFT_LED,
  }, {
    .gpio_pin = D4,
    .dir = DIR_FRONT_LED,
    .sector_width = SECTOR_FRONT_LED,
  }, {
    .gpio_pin = D6,
    .dir = DIR_FRONT_RIGHT_LED,
    .sector_width = SECTOR_FRONT_RIGHT_LED,
  }, {
    .gpio_pin = D5,
    .dir = DIR_BACK_RIGHT_LED,
    .sector_width = SECTOR_BACK_RIGHT_LED,
  },
};


static bool hiawatha()
{
  uint16_t val = 1;
  uint8_t *one = (uint8_t *)&val;

  return *one;
}

uint32_t swap32(uint32_t orig)
{
  return ((orig & 0xFF000000) >> 24) | ((orig & 0x00FF0000) >> 8) |
         ((orig & 0x0000FF00) << 8) | ((orig & 0x000000FF) << 24);  
}

uint16_t swap16(uint16_t orig)
{
  return (uint16_t)(((orig & 0xFF00u) >> 8) | ((orig & 0x00FFu) << 8));
}

uint16_t tobe16(uint16_t orig)
{
  if (hiawatha())
      return swap16(orig);

  return orig;
}

uint32_t tobe32(uint32_t orig)
{
  if (hiawatha())
      return swap32(orig);

  return orig;
}

static unsigned int g_state;
static unsigned short g_direction;
static unsigned short g_dest_dir;
static unsigned int g_distance;

#define WIRE_PORT Wire
#define ICM_AD0_VAL 1

static ICM_20948_I2C g_icm;
static bool g_icm_ready;
static float g_roll_rad;
static float g_pitch_rad;
static uint32_t g_last_heading_ms;
static float g_gyr_bias_x;
static float g_gyr_bias_y;
static float g_gyr_bias_z;
static bool g_heading_init_done;
static float g_mag_min_x;
static float g_mag_min_y;
static float g_mag_min_z;
static float g_mag_max_x;
static float g_mag_max_y;
static float g_mag_max_z;
static float g_mag_off_x;
static float g_mag_off_y;
static float g_mag_off_z;
static float g_mag_scale_x = 1.0f;
static float g_mag_scale_y = 1.0f;
static float g_mag_scale_z = 1.0f;
static enp_cap_state_t g_cap_state = ENP_CAP_STATE_UNKNOWN;

#define MAG_CAL_TIMEOUT_MS 18000U
#define MAG_CAL_MAX_SAMPLES 18000U
#define MAG_CAL_MIN_SAMPLES 200U
#define MAG_CAL_MIN_HALF_RANGE 1.0e-3f
#define IMU_INIT_TIMEOUT_MS 2000U
#define IMU_INIT_MIN_SAMPLES 60U
#define CAL_STATE_SWITCH_DELAY_MS 1500U

static void set_all_dir_leds(bool on);

// Matches app/src/main/java/com/example/lakki_phone/bluetooth/BleGattClient.kt
static BLEUUID SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID RX_CHAR_UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"); // phone -> cap (WRITE)
static BLEUUID TX_CHAR_UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"); // cap -> phone (NOTIFY)

BLEServer* pServer = nullptr;
BLEService* pService = nullptr;
BLECharacteristic* pRxCharacteristic = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;

volatile bool deviceConnected = false;
volatile bool previouslyConnected = false;


void msg_send(void *msg, unsigned int size)
{
  /* This is not atomic... */
  if (!deviceConnected)
    return;
  //Serial.printf("Sending msg %p, %u\n", msg, size);
  pTxCharacteristic->setValue((uint8_t *)msg, size);
  pTxCharacteristic->notify();
}

static size_t append_text_attr(uint8_t *buf, size_t max_len, const char *text)
{
  size_t text_len;
  uint16_t attr_len;

  if (!text || !text[0] || max_len < 4)
    return 0;

  text_len = strlen(text);
  if (text_len > (max_len - 4))
    text_len = max_len - 4;

  attr_len = (uint16_t)(4 + text_len);
  uint16_t be_attr_type = tobe16((uint16_t)ENP_ATTRIBUTE_TYPE_TEXT_UTF8);
  uint16_t be_attr_len = tobe16(attr_len);

  memcpy(&buf[0], &be_attr_type, sizeof(be_attr_type));
  memcpy(&buf[2], &be_attr_len, sizeof(be_attr_len));
  memcpy(&buf[4], text, text_len);

  return attr_len;
}

static void cap_state_send(enp_cap_state_t state, const char *info)
{
  uint8_t msg[192] = {0};
  size_t payload_len = 0;
  struct msg_header *hdr = (struct msg_header *)msg;
  enp_cap_state_header_t *state_hdr = (enp_cap_state_header_t *)(msg + sizeof(*hdr));

  payload_len = append_text_attr(msg + sizeof(*hdr) + sizeof(*state_hdr),
                                 sizeof(msg) - sizeof(*hdr) - sizeof(*state_hdr),
                                 info);

  hdr->type = tobe32(ENP_MESSAGE_TYPE_CAP_STATE);
  hdr->msg_len = tobe32((uint32_t)(sizeof(*hdr) + sizeof(*state_hdr) + payload_len));
  state_hdr->state = tobe32((uint32_t)state);
  state_hdr->reserved = 0;

  msg_send(msg, sizeof(*hdr) + sizeof(*state_hdr) + payload_len);
}

static void cap_state_set(enp_cap_state_t state, const char *info)
{
  if (g_cap_state == state && (!info || !info[0]))
    return;

  g_cap_state = state;
  cap_state_send(state, info);
}

static void debug_log_send(uint32_t severity, const char *line)
{
  uint8_t msg[192] = {0};
  size_t payload_len = 0;
  struct msg_header *hdr = (struct msg_header *)msg;
  enp_debug_log_header_t *dbg_hdr = (enp_debug_log_header_t *)(msg + sizeof(*hdr));

  payload_len = append_text_attr(msg + sizeof(*hdr) + sizeof(*dbg_hdr),
                                 sizeof(msg) - sizeof(*hdr) - sizeof(*dbg_hdr),
                                 line);

  hdr->type = tobe32(ENP_MESSAGE_TYPE_DEBUG_LOG);
  hdr->msg_len = tobe32((uint32_t)(sizeof(*hdr) + sizeof(*dbg_hdr) + payload_len));
  dbg_hdr->severity = tobe32(severity);
  dbg_hdr->reserved = 0;

  msg_send(msg, sizeof(*hdr) + sizeof(*dbg_hdr) + payload_len);
}

static void ble_debug_logf(const char *fmt, ...)
{
  char line[160];
  va_list args;

  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);

  debug_log_send(0, line);
}


static bool has_sane_mag_scaling(float rx, float ry, float rz)
{
  return rx > MAG_CAL_MIN_HALF_RANGE &&
         ry > MAG_CAL_MIN_HALF_RANGE &&
         rz > MAG_CAL_MIN_HALF_RANGE;
}

static int apply_declination_deg(int dir)
{
  /*
   * The 'Lakki' is mostly going to be used in known location.
   * Let's use a fixed declination matching that location for now.
   * TODO: Think of a way to compute the declination on mobile application,
   * based on the GPS location, and send the declination information
   * via new BLE message when navigation is started.
   */
  int enontekio_declination = 15;

  dir -= enontekio_declination;

  while (dir < 0)
    dir += 360;

  while (dir >= 360)
    dir -= 360;

  return dir;
}

static unsigned int head2deg(float heading)
{
  heading *= 180/M_PI;

  return (int)heading;
}

static float wrap_pi(float rad)
{
  while (rad > PI)
    rad -= 2.0f * PI;
  while (rad < -PI)
    rad += 2.0f * PI;

  return rad;
}

static bool setup_compass()
{
  bool initialized = false;

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  for (int i = 0; i < 10 && !initialized; i++) {
    g_icm.begin(WIRE_PORT, ICM_AD0_VAL);
    if (g_icm.status == ICM_20948_Stat_Ok)
      initialized = true;
    else
      delay(200);
  }

  if (!initialized) {
    Serial.print("[ICM] Init failed: ");
    Serial.println(g_icm.statusString());
    cap_state_set(ENP_CAP_STATE_ERROR, "ICM init failed");
    return false;
  }

  g_icm.swReset();
  delay(250);
  g_icm.sleep(false);
  g_icm.lowPower(false);
  g_icm.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);

  ICM_20948_fss_t fss;
  fss.a = gpm2;
  fss.g = dps250;
  g_icm.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);

  ICM_20948_dlpcfg_t dlpcfg;
  dlpcfg.a = acc_d111bw4_n136bw;
  dlpcfg.g = gyr_d119bw5_n154bw3;
  g_icm.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
  g_icm.enableDLPF(ICM_20948_Internal_Acc, true);
  g_icm.enableDLPF(ICM_20948_Internal_Gyr, true);

  g_icm.startupMagnetometer();
  if (g_icm.status != ICM_20948_Stat_Ok) {
    Serial.print("[ICM] Magnetometer startup failed: ");
    Serial.println(g_icm.statusString());
    cap_state_set(ENP_CAP_STATE_ERROR, "Magnetometer startup failed");
    return false;
  }

  Serial.println("[ICM] Keep cap stationary, collecting gyro/accel baseline...");
  set_all_dir_leds(true);

  float acc_roll_sum = 0.0f;
  float acc_pitch_sum = 0.0f;
  float gyr_x_sum = 0.0f;
  float gyr_y_sum = 0.0f;
  float gyr_z_sum = 0.0f;
  uint32_t sample_count = 0;
  const uint32_t init_start_ms = millis();

  while ((millis() - init_start_ms) < IMU_INIT_TIMEOUT_MS) {
    if (!g_icm.dataReady()) {
      delay(2);
      continue;
    }

    g_icm.getAGMT();

    const float ax = g_icm.accX();
    const float ay = g_icm.accY();
    const float az = g_icm.accZ();

    acc_roll_sum += atan2(ay, az);
    acc_pitch_sum += atan2(-ax, sqrtf((ay * ay) + (az * az)));
    gyr_x_sum += g_icm.gyrX() * DEG_TO_RAD;
    gyr_y_sum += g_icm.gyrY() * DEG_TO_RAD;
    gyr_z_sum += g_icm.gyrZ() * DEG_TO_RAD;

    sample_count++;
  }

  set_all_dir_leds(false);

  if (sample_count < IMU_INIT_MIN_SAMPLES) {
    Serial.printf("[ICM] Baseline init failed: only %lu samples\n", sample_count);
    cap_state_set(ENP_CAP_STATE_ERROR, "IMU baseline initialization failed");
    return false;
  }

  g_roll_rad = acc_roll_sum / sample_count;
  g_pitch_rad = acc_pitch_sum / sample_count;
  g_gyr_bias_x = gyr_x_sum / sample_count;
  g_gyr_bias_y = gyr_y_sum / sample_count;
  g_gyr_bias_z = gyr_z_sum / sample_count;
  g_heading_init_done = true;

  Serial.println("[ICM] Compass ready");
  Serial.printf("[ICM] Init samples=%lu, gyro bias(rad/s)=%.5f, %.5f, %.5f\n",
                sample_count, g_gyr_bias_x, g_gyr_bias_y, g_gyr_bias_z);
  g_last_heading_ms = millis();

  return true;
}

static void set_all_dir_leds(bool on)
{
  int i;

  for (i = 0; i < NUM_LEDS; i++)
    digitalWrite(g_led_arr[i].gpio_pin, on ? HIGH : LOW);
}

static void indicate_fault_all_leds()
{
  /* Keep all direction LEDs lit for clear user-visible warning. */
  set_all_dir_leds(true);
  delay(5000);
  set_all_dir_leds(false);
}

static void calibrate_magnetometer()
{
  uint32_t samples = 0;
  const uint32_t start_ms = millis();
  uint32_t last_blink_toggle_ms = start_ms;
  bool leds_on = false;

  if (!g_icm_ready)
    return;

  g_mag_min_x = g_mag_min_y = g_mag_min_z = INFINITY;
  g_mag_max_x = g_mag_max_y = g_mag_max_z = -INFINITY;

  cap_state_set(ENP_CAP_STATE_CALIBRATING, 0);
  Serial.println("[CAL] Magnetometer calibration start");
  Serial.printf("[CAL] Hold still, calibration mode switches in %u ms...\n", CAL_STATE_SWITCH_DELAY_MS);
  set_all_dir_leds(true);
  delay(CAL_STATE_SWITCH_DELAY_MS);
  set_all_dir_leds(false);

  Serial.println("[CAL] Move cap now");
  Serial.println("[CAL] Move cap in figure-8 and full rotations...");

  while ((millis() - start_ms) < MAG_CAL_TIMEOUT_MS && samples < MAG_CAL_MAX_SAMPLES) {
    const uint32_t now = millis();

    if ((now - last_blink_toggle_ms) >= 200) {
      leds_on = !leds_on;
      set_all_dir_leds(leds_on);
      last_blink_toggle_ms = now;
    }

    if (!g_icm.dataReady()) {
      delay(2);
      continue;
    }

    g_icm.getAGMT();

    const float mx = g_icm.magX();
    const float my = g_icm.magY();
    const float mz = g_icm.magZ();

    if (mx < g_mag_min_x)
      g_mag_min_x = mx;
    if (my < g_mag_min_y)
      g_mag_min_y = my;
    if (mz < g_mag_min_z)
      g_mag_min_z = mz;

    if (mx > g_mag_max_x)
      g_mag_max_x = mx;
    if (my > g_mag_max_y)
      g_mag_max_y = my;
    if (mz > g_mag_max_z)
      g_mag_max_z = mz;

    samples++;

    if ((samples % 100) == 0) {
      Serial.printf("[CAL] samples=%lu elapsed=%lums\n", samples, now - start_ms);
    }
  }

  set_all_dir_leds(false);

  if (samples < MAG_CAL_MIN_SAMPLES) {
    Serial.printf("[CAL] Warning: only %lu samples, calibration weak. Offsets left at 0.\n", samples);
    cap_state_set(ENP_CAP_STATE_ERROR, "Magnetometer calibration had too few samples");
    indicate_fault_all_leds();
    return;
  }

  const float rx = (g_mag_max_x - g_mag_min_x) * 0.5f;
  const float ry = (g_mag_max_y - g_mag_min_y) * 0.5f;
  const float rz = (g_mag_max_z - g_mag_min_z) * 0.5f;

  if (!has_sane_mag_scaling(rx, ry, rz)) {
    Serial.printf("[CAL] Invalid ranges rx=%.6f ry=%.6f rz=%.6f, keeping previous calibration\n", rx, ry, rz);
    cap_state_set(ENP_CAP_STATE_ERROR, "Magnetometer calibration failed due to invalid ranges");
    indicate_fault_all_leds();
    return;
  }

  const float r_avg = (rx + ry + rz) / 3.0f;

  g_mag_off_x = (g_mag_max_x + g_mag_min_x) * 0.5f;
  g_mag_off_y = (g_mag_max_y + g_mag_min_y) * 0.5f;
  g_mag_off_z = (g_mag_max_z + g_mag_min_z) * 0.5f;
  g_mag_scale_x = r_avg / rx;
  g_mag_scale_y = r_avg / ry;
  g_mag_scale_z = r_avg / rz;

  if (!has_sane_mag_scaling(g_mag_scale_x, g_mag_scale_y, g_mag_scale_z)) {
    Serial.println("[CAL] Invalid computed scales, keeping previous calibration");
    cap_state_set(ENP_CAP_STATE_ERROR, "Magnetometer calibration failed due to invalid scales");
    indicate_fault_all_leds();
    return;
  }

  Serial.printf("[CAL] done samples=%lu duration=%lums\n", samples, millis() - start_ms);
  Serial.printf("[CAL] min=(%.2f, %.2f, %.2f) max=(%.2f, %.2f, %.2f)\n",
                g_mag_min_x, g_mag_min_y, g_mag_min_z,
                g_mag_max_x, g_mag_max_y, g_mag_max_z);
  Serial.printf("[CAL] offsets=(%.2f, %.2f, %.2f)\n", g_mag_off_x, g_mag_off_y, g_mag_off_z);
  Serial.printf("[CAL] half-ranges=(%.3f, %.3f, %.3f) avg=%.3f\n", rx, ry, rz, r_avg);
  Serial.printf("[CAL] scales=(%.3f, %.3f, %.3f)\n", g_mag_scale_x, g_mag_scale_y, g_mag_scale_z);
  cap_state_set(ENP_CAP_STATE_NAVIGATING, 0);
}

static void update_heading()
{
  if (!g_icm_ready)
    return;

  if (!g_heading_init_done)
    return;

  if (!g_icm.dataReady())
    return;

  g_icm.getAGMT();

  const float ax = g_icm.accX();
  const float ay = g_icm.accY();
  const float az = g_icm.accZ();
  const float gx = g_icm.gyrX() * DEG_TO_RAD - g_gyr_bias_x;
  const float gy = g_icm.gyrY() * DEG_TO_RAD - g_gyr_bias_y;
  const float mx = g_icm.magX();
  const float my = g_icm.magY();
  const float mz = g_icm.magZ();
  const float mx_n = (mx - g_mag_off_x) * g_mag_scale_x;
  const float my_n = (my - g_mag_off_y) * g_mag_scale_y;
  const float mz_n = (mz - g_mag_off_z) * g_mag_scale_z;

  const uint32_t now = millis();
  float dt = (now - g_last_heading_ms) / 1000.0f;
  if (dt <= 0.0f || dt > 0.2f)
    dt = 0.01f;
  g_last_heading_ms = now;

  const float acc_roll = atan2(ay, az);
  const float acc_pitch = atan2(-ax, sqrtf((ay * ay) + (az * az)));

  g_roll_rad = wrap_pi(0.98f * (g_roll_rad + gx * dt) + 0.02f * acc_roll);
  g_pitch_rad = wrap_pi(0.98f * (g_pitch_rad + gy * dt) + 0.02f * acc_pitch);

  const float sin_roll = sinf(g_roll_rad);
  const float cos_roll = cosf(g_roll_rad);
  const float sin_pitch = sinf(g_pitch_rad);
  const float cos_pitch = cosf(g_pitch_rad);

  const float mag_x_h = mx_n * cos_pitch + mz_n * sin_pitch;
  const float mag_y_h = mx_n * sin_roll * sin_pitch + my_n * cos_roll - mz_n * sin_roll * cos_pitch;

  float heading = atan2f(-mag_x_h, mag_y_h);
  if (heading < 0.0f)
    heading += 2.0f * PI;

  g_direction = apply_declination_deg(head2deg(heading));

  if (debug) {
    Serial.printf("[ICM] dir=%u, roll=%0.2f, pitch=%0.2f\n", g_direction,
                  g_roll_rad * RAD_TO_DEG, g_pitch_rad * RAD_TO_DEG);
  }
}

class CapServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    deviceConnected = true;
    Serial.println("[BLE] Phone connected");

    // Optional: try larger MTU for larger app payload framing.
    // Android side already handles MTU changes if they happen.
    // server->updatePeerMTU(server->getConnId(), 185);
  }

  void onDisconnect(BLEServer* server) override {
    deviceConnected = false;
    Serial.println("[BLE] Phone disconnected");
  }
};

enum lakki_state {
  STATE_INIT,
  HANDSHAKE_RECVD,
  DEST_SET,
  SEND_CAP_DIR,
  TURN_OFF_LEDS,
};

static void add_state(unsigned int state) {
  g_state |= 1 << state;
}

static void del_state(unsigned int state) {
  g_state &= ~(1 << state);
}

static int handle_handshake(void *data, unsigned int msglen)
{
  add_state(HANDSHAKE_RECVD);
  Serial.println("Handshake recv'd");
/*  pTxCharacteristic->setValue((uint8_t*)value.data(), value.size());
      pTxCharacteristic->notify();
      */
  return 0;
}

static int handle_movement(void *data, unsigned int msglen)
{
  Serial.println("Movement recv'd");
  return 0;
}

static int handle_dest(void *data, unsigned int msglen)
{
  enp_destination_header *hdr = (enp_destination_header *)data;

  g_dest_dir = tobe32(hdr->direction);
  g_distance = tobe32(hdr->distance_meters);
  add_state(DEST_SET);

  Serial.printf("Dest recv'd, dir %u, distance %u\n",g_dest_dir, g_distance);
  return 0;
}

static int handle_dest_req(void *data, unsigned int msglen)
{
  Serial.println("Dest REQ?? Why did I get this?");
  return 0;
}

static int handle_cap_dir(void *data, unsigned int msglen)
{
  Serial.println("Cap DIR?? Why did I get this?");
  return 0;
}

static int handle_cap_dir_start(void *data, unsigned int msglen)
{
  Serial.println("Cap Dir Start");
   add_state(SEND_CAP_DIR);
  return 0;
}

static int handle_cap_dir_stop(void *data, unsigned int msglen)
{
  Serial.println("Cap Dir Stop");
     del_state(SEND_CAP_DIR);
  return 0;
}

struct msg_handlers {
  int (*handler)(void *data, unsigned int msglen);
  unsigned int msg_min_len;
};

static const struct msg_handlers g_handlers[] = {
  {0}, /* Invalid*/
  /* [ENP_MESSAGE_TYPE_HANDSHAKE] = */{
    .handler = &handle_handshake,
    .msg_min_len = sizeof(enp_handshake_header),
  },
  /*[ENP_MESSAGE_TYPE_DESTINATION] = */{
    .handler = &handle_dest,
    .msg_min_len = sizeof(enp_destination_header),
  },
  /*[ENP_MESSAGE_TYPE_MOVEMENT] = */{
    .handler = &handle_movement,
    .msg_min_len = sizeof(enp_movement_header),
  },
  /*[ENP_MESSAGE_TYPE_DESTINATION_REQUEST] = */{
    .handler = &handle_dest_req,
    .msg_min_len = sizeof(enp_destination_request_header),
  },
  /*[ENP_MESSAGE_TYPE_CAP_DIRECTION] = */{
    .handler = &handle_cap_dir,
    .msg_min_len = sizeof(enp_cap_direction_header),
  },
  /*[ENP_MESSAGE_TYPE_CAP_DIRECTION_REQUEST_START] = */{
    .handler = &handle_cap_dir_start,
    .msg_min_len = sizeof(enp_cap_direction_request_header),
  },
  /*[ENP_MESSAGE_TYPE_CAP_DIRECTION_REQUEST_STOP] = */{
    .handler = &handle_cap_dir_stop,
    .msg_min_len = sizeof(enp_cap_direction_request_header),
  },
};

class CapRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    struct msg_header *hdr;
    void *data = characteristic->getData();
    unsigned int data_len = characteristic->getLength();
    unsigned int handled = 0;

    Serial.printf("Char len %u\n", data_len);

    while (handled < data_len) {
      uint32_t type_le;
      uint32_t len_le;
      static const struct msg_handlers *msg_handler;
      /*
       * The data types must be continuous. If some types aren't handled we should change
       * jump-table to switch-case
       */
      if (data_len - handled < sizeof(msg_header))
        return;
      
      hdr =  (struct msg_header*)(((uint8_t *)data) + handled);
      if (!hdr->msg_len)
        return;

      type_le = tobe32(hdr->type);
      len_le = tobe32(hdr->msg_len);

      if (len_le > data_len) {
        Serial.printf("Bad Data!\n");
        Serial.printf("MSG type 0x%x, len %u\n", type_le, len_le);
        return;
      }

      Serial.printf("MSG type %u, len %u\n", type_le, len_le);

      if (ENP_MESSAGE_TYPE_INVALID >= type_le ||
          ENP_MESSAGE_TYPE_CAP_DIRECTION_REQUEST_STOP < type_le)
        goto out_handled;

      msg_handler = &g_handlers[type_le];

      if (len_le - sizeof(*hdr) < msg_handler->msg_min_len) {
        Serial.println("MSG too short");
        goto out_handled;
      }

      msg_handler->handler(MSG_PAYLOAD(hdr), len_le - sizeof(msg_header));

out_handled:
      handled += len_le;
    }
  }
};

void setupAdvertising() {
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);

  // Common compatibility hint values for Android BLE central devices.
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();
  Serial.println("[BLE] Advertising started");
}

void blink(int pin, int numblink)
{
  int i;

   Serial.printf("Blink LED %d, %d times\n", pin, numblink);

  for (i = 0; i < numblink; i++){
    digitalWrite(pin, HIGH);
    delay(300);
    digitalWrite(pin, LOW);
    delay(300);
  }
}

void setup_led_gpios()
{
  int i;
  for (i = 0; i < NUM_LEDS; i++) {
    const struct mva_led *led = &g_led_arr[i];

    pinMode(led->gpio_pin, OUTPUT);
    blink(led->gpio_pin, i + 1);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  delay(3000);
  Serial.println("[SYS] Boot");

  setup_led_gpios();
  g_icm_ready = setup_compass();
  if (!g_icm_ready)
    cap_state_set(ENP_CAP_STATE_ERROR, "Compass setup failed");
  calibrate_magnetometer();

  BLEDevice::init("LakkiCap");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new CapServerCallbacks());

  pService = pServer->createService(SERVICE_UUID);

  // RX characteristic: app writes commands with WRITE_TYPE_DEFAULT.
  pRxCharacteristic = pService->createCharacteristic(
      RX_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(new CapRxCallbacks());

  // TX characteristic: cap notifies app.
  pTxCharacteristic = pService->createCharacteristic(
      TX_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  setupAdvertising();
}

static bool is_leds_off_set()
{
  return (g_state & (1 << TURN_OFF_LEDS));
}

static bool is_dest_set()
{
  return (g_state & (1 << DEST_SET));
}
static bool is_handshake_recvd()
{
  return (g_state & (1 << HANDSHAKE_RECVD));
}

static bool is_cap_dir_send_en()
{
  return (g_state & (1 << SEND_CAP_DIR));
}

static void handshake_reply()
{
  /* Send handshake msg */
  del_state(HANDSHAKE_RECVD);
}

static bool is_get_dir_set()
{
  return g_icm_ready && (is_cap_dir_send_en() || is_dest_set());
}

static void litemup()
{
  unsigned short dir = g_direction;
  int i;

  for (i = 0; i < NUM_LEDS; i++) {
    const struct mva_led *led = &g_led_arr[i];
    unsigned short sector_left, sector_right;
    unsigned short half_sector = led->sector_width / 2;

    if (led->dir == 0)
      sector_left = 360 - half_sector;
    else
      sector_left = led->dir - half_sector;

    sector_right = led->dir + half_sector;

    if ((led->dir && dir > sector_left && dir < sector_right) ||
        (!led->dir &&
          (
           (dir > sector_left && dir <= 360) ||
           (dir < sector_right))))
      digitalWrite(led->gpio_pin, HIGH);
    else
      digitalWrite(led->gpio_pin, LOW);
  }
}

static void leds_off()
{
  int i;
  for (i = 0; i < NUM_LEDS; i++)
    digitalWrite(g_led_arr[i].gpio_pin, LOW);

  del_state(TURN_OFF_LEDS);
}

static void show_destination()
{
  /* Loop counter for keeping LEDs lit for LED_IND_LOOPS loops*/
  static int ctr = LED_IND_LOOPS;

  ctr--;
  if (ctr <= 0) {
    ctr = LED_IND_LOOPS;
    del_state(DEST_SET);
    add_state(TURN_OFF_LEDS);
  }
  /*
   * Turn off all LED's except the LED to show correct direction.
   * Lit correct direction LED.
   */
   litemup();
  return;
}

struct cap_dir_msg {
  struct msg_header hdr;
  enp_cap_direction_header_t cdh;
};

static void cap_dir_send()
{
  struct cap_dir_msg msg;
  static uint32_t lastMs = 0;
  const uint32_t now = millis();

  if ((now - lastMs) >= 100) {
    lastMs = now;
    msg.hdr.type = tobe32(ENP_MESSAGE_TYPE_CAP_DIRECTION);
    msg.hdr.msg_len = tobe32(sizeof(cap_dir_msg));
    msg.cdh.direction = tobe32(g_direction);
  
    msg_send(&msg, sizeof(msg));

  }
    /* Send cap-dir message */
}

static void state_machine()
{
  if (is_get_dir_set())
  {
    update_heading();
  }
  if (is_leds_off_set())
  {
    leds_off();
  }
  if (is_dest_set()) {
      /* Light destination LED(s) */
      show_destination();
  }
  if (is_handshake_recvd()) {
    handshake_reply();
  }
  if (is_cap_dir_send_en()) {
    cap_dir_send();
  }
#if ENABLE_BLE_DIRECTION_DEBUG
  {
    static uint32_t lastDbgMs = 0;
    uint32_t now = millis();

    if ((now - lastDbgMs) >= 1000) {
      lastDbgMs = now;
      ble_debug_logf("cap direction: %u deg", g_direction);
    }
  }
#endif
}

void loop() {
  // Restart advertising after disconnect (if needed).
  if (!deviceConnected && previouslyConnected) {
    delay(150);
    BLEDevice::startAdvertising();
    Serial.println("[BLE] Restarted advertising after disconnect");
    previouslyConnected = deviceConnected;
  }

  // Connection edge.
  if (deviceConnected && !previouslyConnected) {
    previouslyConnected = deviceConnected;
  }

  state_machine();

  delay(10);
}
