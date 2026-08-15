/*
 * ============================================================================
 *  Node 1  (OUTDOOR SENSOR)  -  ESP32-C3
 * ============================================================================
 *  No internet, no router, does not even need the WiFi password.
 *  It only needs two facts: which MAC to send to (NODE0_MAC) and which radio
 *  channel to use (WIFI_CHANNEL).
 *
 *  One work cycle (awake ~5 seconds, asleep the rest of the time)
 *      wake --> startEspNow()  power the radio, lock the channel, register Node 0
 *           --> doOneCycle()  --> readAveraged()  read the DHT 3x and average
 *                             --> esp_now_send()  transmit to Node 0
 *                             --> onEspNowSent()  did it arrive or not
 *           --> deep sleep for INTERVAL_MIN, then wake and repeat
 *
 *  ** Flash NODE 0 first, then copy its NODE0_MAC / WIFI_CHANNEL down here **
 *  ---------------------------------------------------------------------------
 *  Function list
 *    onEspNowSent()   [callback] transmission finished - did it reach the peer
 *    startEspNow()    power the radio, lock the channel, register Node 0 as peer
 *    readAveraged()   read the DHT 3x, discard failures, average what is left
 *    doOneCycle()     read -> send -> wait for result -> report
 *  ---------------------------------------------------------------------------
 *  setup()  Runs again in full on every wake-up, not once like on Node 0
 *      1. dht.begin()        prepare the sensor
 *      2. startEspNow()      power the radio, lock the channel, register Node 0
 *      3. doOneCycle()       read 3x -> average -> send -> wait   (~5 seconds)
 *      4. esp_deep_sleep()   sleep, then wake and run steps 1-4 again
 *
 *  loop()   ***never runs at all*** while USE_DEEP_SLEEP is 1, because setup()
 *           falls asleep before reaching it. It is only used when
 *           USE_DEEP_SLEEP is 0: it counts time and calls doOneCycle().
 *
 *  Deep sleep wipes RAM, so every variable starts from zero on each cycle.
 *  Anything that must survive a sleep has to be declared RTC_DATA_ATTR.
 *  ---------------------------------------------------------------------------
 *  Board : ESP32C3 Dev Module   (Arduino-ESP32 core 3.x)
 *  Libs  : DHT sensor library (Adafruit) + Adafruit Unified Sensor
 * ============================================================================
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <DHT.h>

//  CONFIG

// --- MAC ADDRESS ---
#define NODE0_MAC     { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF }   // who to send to
#define WIFI_CHANNEL  1                      // must match Node 0 or they cannot hear each other

// --- SENSOR ---
#define DHT_PIN       4                      // data pin
#define DHT_TYPE      DHT22                  // set to DHT11 or DHT22 to match your part

// --- Reporting interval ---
#define INTERVAL_MIN  15                     // pick 1 / 5 / 15 / 30 / 60 minutes

// --- 1 : SLEEP, 0 NONE SLEEP  ---
#define USE_DEEP_SLEEP  1                    // use 0 for bench testing

// ===========================================================================
#define SAMPLE_COUNT    3                    // read 3 times and average
#define SAMPLE_GAP_MS   2200                 // DHT22 needs >2 s between reads or it repeats the old value

// ESP-NOW packet  ***  must match node0_root.ino exactly  ***
typedef struct __attribute__((packed)) {
  float   temperature;
  float   humidity;
  uint8_t validSamples;                      // how many of the 3 reads worked (0 = sensor fault)
} Packet;

static uint8_t node0Mac[6] = NODE0_MAC;

DHT dht(DHT_PIN, DHT_TYPE);

volatile bool sendDone = false;              // has the callback fired yet
volatile bool sendOk   = false;              // what was the result
uint32_t      lastRun  = 0;                  // only used when deep sleep is off

// ---------------------------------------------------------------------------
// ESP-NOW
// ---------------------------------------------------------------------------

// [callback] Transmission finished. status says whether Node 0 sent back a
// MAC-layer ACK. It runs in a different task from doOneCycle(), so the result
// comes back through volatile variables.
void onEspNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  sendOk   = (status == ESP_NOW_SEND_SUCCESS);
  sendDone = true;
}

// Power the radio without joining the router, pin it to WIFI_CHANNEL, and
// register Node 0 as the destination. Joining a router would let the driver
// force its own channel over ours, and ESP-NOW would then transmit on the
// wrong one.
void startEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);             // leave the router, but keep the radio powered

  esp_wifi_set_promiscuous(true);            // standard trick to make the channel change stick
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {            // must always come after WiFi.mode()
    Serial.println("[ESPNOW] init failed -> restarting");
    delay(2000);
    ESP.restart();
  }
  esp_now_register_send_cb(onEspNowSent);

  // ESP-NOW refuses to send to a MAC that is not in the peer table
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, node0Mac, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  peer.ifidx   = WIFI_IF_STA;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESPNOW] add peer failed -> check WIFI_CHANNEL / WiFi mode");
    delay(2000);
    ESP.restart();
  }

  Serial.printf("[ESPNOW] ready  ch=%d  sending to %02X:%02X:%02X:%02X:%02X:%02X\n",
                WIFI_CHANNEL, node0Mac[0], node0Mac[1], node0Mac[2],
                node0Mac[3], node0Mac[4], node0Mac[5]);
}

// ---------------------------------------------------------------------------
// SENSOR
// ---------------------------------------------------------------------------

// Read the DHT 3 times, discard failed or out-of-range readings, and average
// only the good ones. Returns how many reads succeeded; temperature and
// humidity come back through the pointers.
// (The DHT fails often, especially while the radio is active, which is exactly
// why the reading is repeated.)
uint8_t readAveraged(float *tOut, float *hOut) {
  float tSum = 0, hSum = 0;
  uint8_t valid = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    if (i > 0) delay(SAMPLE_GAP_MS);
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    bool ok = !isnan(h) && !isnan(t) &&
              h >= 0.0f && h <= 100.0f && t > -40.0f && t < 85.0f;

    Serial.printf("[DHT] read %d : ", i + 1);
    if (ok) {
      Serial.printf("T=%.2f C  H=%.2f %%\n", t, h);
      tSum += t; hSum += h; valid++;
    } else {
      Serial.println("failed (skipped)");
    }
  }

  if (valid) {
    *tOut = tSum / valid;
    *hOut = hSum / valid;
    Serial.printf("[DHT] average of %u reads : T=%.2f C  H=%.2f %%\n", valid, *tOut, *hOut);
  } else {
    *tOut = *hOut = 0.0f;                    // zeros plus validSamples=0 tell the server the sensor is faulty
    Serial.println("[DHT] all 3 reads failed");
  }
  return valid;
}

// One work cycle: read -> send -> wait for the result -> report.
// esp_now_send() returns immediately without knowing the outcome; the real
// result arrives later through onEspNowSent(). Hence the wait loop - and the
// delay() inside it is what yields the CPU so the WiFi task can fire that callback.
void doOneCycle() {
  Packet p;
  p.validSamples = readAveraged(&p.temperature, &p.humidity);

  sendDone = false;
  esp_now_send(node0Mac, (uint8_t *)&p, sizeof(p));

  uint32_t t0 = millis();
  while (!sendDone && millis() - t0 < 300) delay(5);

  if (!sendDone)   Serial.println("[TX ] no result returned");
  else if (sendOk) Serial.println("[TX ] delivered");
  else             Serial.println("[TX ] did not reach Node 0 (check MAC / channel / range)");
}

// ---------------------------------------------------------------------------

// Do one cycle, then sleep. Waking up re-enters here from the top, with no
// state carried over from the previous cycle.
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\n\n=== Node 1 (Outdoor Sensor) - reporting every %d min ===\n", INTERVAL_MIN);

  dht.begin();
  startEspNow();
  doOneCycle();

#if USE_DEEP_SLEEP
  Serial.printf("[SLEEP] deep sleep for %d min\n\n", INTERVAL_MIN);
  Serial.flush();                            // let the serial buffer drain or the text is lost
  esp_deep_sleep((uint64_t)INTERVAL_MIN * 60ULL * 1000000ULL);
  // nothing past this line ever executes
#else
  lastRun = millis();
  Serial.printf("[WAIT] next cycle in %d min\n\n", INTERVAL_MIN);
#endif
}

// Only used when USE_DEEP_SLEEP is 0: stay awake and count time with millis()
// instead of the RTC timer.
void loop() {
#if !USE_DEEP_SLEEP
  if (millis() - lastRun >= (uint32_t)INTERVAL_MIN * 60000UL) {
    doOneCycle();
    lastRun = millis();
    Serial.printf("[WAIT] next cycle in %d min\n\n", INTERVAL_MIN);
  }
  delay(100);
#endif
}
