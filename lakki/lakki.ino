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

#include "external_navigation_protocol.h"

#define LED_IND_LOOPS 1000;


static const int debug = 0;

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

enum lakki_state {
  STATE_INIT,
  HANDSHAKE_RECVD,
  DEST_SET,
  SEND_CAP_DIR,
  TURN_OFF_LEDS,
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

static void add_state(lakki_state state) {
  g_state |= 1 << state;
}

static void del_state(lakki_state state) {
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
    /*
     * Make a copy as an attempt to not access something
     * which may change under the hood. This may not be safe.
     * (I've no idea if characteristic contains pointers)
     */
 //   BLECharacteristic c = *characteristic;
    struct msg_header *hdr;
    void *data = characteristic->getData();
    unsigned int data_len = characteristic->getLength();
//    enp_message_type_t *type;
//    uint32_t *msg_len;
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
//      type = (enp_message_type_t *)
//      msg_len = (((uint32_t *)type) + 1);
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
    /*
    std::string value = characteristic->getValue();
    if (value.empty()) {
      return;
    }

    Serial.printf("[BLE] RX %u bytes: ", (unsigned)value.size());
    for (size_t i = 0; i < value.size(); ++i) {
      Serial.printf("%02X ", (uint8_t)value[i]);
    }
    Serial.println();

    // TODO: parse app frames and execute command.
    // Example echo ACK with same payload:
    if (deviceConnected && pTxCharacteristic != nullptr) {
      pTxCharacteristic->setValue((uint8_t*)value.data(), value.size());
      pTxCharacteristic->notify();
    }
    */
  }
};

void msg_send(void *msg, unsigned int size)
{
  /* This is not atomic... */
  if (!deviceConnected)
    return;
  //Serial.printf("Sending msg %p, %u\n", msg, size);
  pTxCharacteristic->setValue((uint8_t *)msg, size);
  pTxCharacteristic->notify();
}

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
    blink(led->gpio_pin, i + 1);/*
    digitalWrite(led->gpio_pin, HIGH);
    delay(500);
    digitalWrite(led->gpio_pin, LOW);
    */
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  delay(3000);
  Serial.println("[SYS] Boot");

  setup_led_gpios();

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

static void litemup()
{
  unsigned short dir = g_direction;
  int i;

  for (i = i; i < NUM_LEDS; i++) {
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

    g_direction += 9;
    if (g_direction >= 360)
      g_direction = 0;
  }
    /* Send cap-dir message */
}

static void state_machine()
{
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

  // Example periodic notification payload (replace with real cap data).
  static uint32_t lastMs = 0;
  const uint32_t now = millis();
  if (deviceConnected && (now - lastMs) >= 1000) {
    lastMs = now;

    // Example payload: 4-byte big-endian int for direction-like telemetry.
    uint8_t payload[4];
    int32_t demoDirection = 90;
    payload[0] = (demoDirection >> 24) & 0xFF;
    payload[1] = (demoDirection >> 16) & 0xFF;
    payload[2] = (demoDirection >> 8) & 0xFF;
    payload[3] = demoDirection & 0xFF;

    pTxCharacteristic->setValue(payload, sizeof(payload));
    pTxCharacteristic->notify();
    // Serial.println("[BLE] TX notify sent");
  }
 // if (debug) {
 //   Serial.println("Loopiti Loopiti");
 //   delay(1000);
 // }
  state_machine();

  delay(10);
}
