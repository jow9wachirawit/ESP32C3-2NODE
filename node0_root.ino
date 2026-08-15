/*
 * ============================================================================
 *  Node 0  (ROOT / GATEWAY)  -  ESP32-C3
 * ============================================================================
 *  The only node with internet access. Acts as a two-way bridge.
 *
 *  Uplink (garden data -> server)
 *      Node 1 --ESP-NOW--> onEspNowRecv() --> handleSensorPacket() --> farm/data
 *
 *  Downlink (server command -> water valve)
 *      farm/cmd --> onMqttMessage() --> relayStart() --> relayWrite() --> valve ON
 *                                          |
 *                            time is up -> loop() calls relayStop() to close it
 *
 *  No logic of its own. The server decides how many seconds to water; this node
 *  just obeys. The shut-off timer runs locally, so the valve still closes on
 *  time even if the network drops mid-cycle.
 *  ---------------------------------------------------------------------------
 *  Function list
 *    relayWrite()          Drive the relay pin and the onboard LED
 *    publishRelay()        Report relay state to the server
 *    relayStart()          Open the valve and record when it must close
 *    relayStop()           Close the valve
 *    onEspNowRecv()        [callback] Node 1 sent sensor data
 *    handleSensorPacket()  Format the data as CSV and publish it
 *    onMqttMessage()       [callback] Server sent a command
 *    mqttTick()            Keep the MQTT connection alive
 *    wifiTick()            Keep the WiFi connection alive
 *  ---------------------------------------------------------------------------
 *  setup()  Runs once at power-on / reset, never again
 *      1. Close the valve first        prevents a watering pulse during boot
 *      2. Connect WiFi                 waits up to 20 s, then wifiTick() takes over
 *      3. Start ESP-NOW                binds onEspNowRecv() to incoming packets
 *      4. Print MAC / channel          copy these into node1_sensor.ino
 *      5. Configure MQTT               binds onMqttMessage() to incoming commands
 *                                      (does not connect here - mqttTick() does that)
 *
 *  loop()   Repeats forever, roughly every 5 ms. Every step is non-blocking,
 *           so nothing can ever delay the valve shut-off.
 *      1. Is it time to close the valve?  -> relayStop()
 *      2. wifiTick()                      is WiFi down?
 *      3. mqttTick()                      is MQTT down? any command waiting?
 *      4. Any pending data from Node 1?   -> handleSensorPacket()
 *
 *  The two callbacks are not part of loop() - the system invokes them itself:
 *      onEspNowRecv()   called by the WiFi task the instant a packet lands
 *      onMqttMessage()  called from inside mqtt.loop() when a message arrives
 *  ---------------------------------------------------------------------------
 *  Board : ESP32C3 Dev Module   (Arduino-ESP32 core 3.x)
 *  Libs  : PubSubClient (Nick O'Leary)
 * ============================================================================
 */

#include <WiFi.h>
#include <esp_now.h>
#include <PubSubClient.h>

#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASS       "YOUR_WIFI_PASSWORD"

#define MQTT_HOST       "192.168.1.100"      // private MQTT broker
#define MQTT_PORT       1883
#define MQTT_USER       ""                   // "" if the broker needs no login
#define MQTT_PASS       ""
#define MQTT_CLIENT_ID  "esp32c3-node0"      // must be unique on the broker

#define TOPIC_DATA      "farm/data"          // uplink   : sensor + status
#define TOPIC_CMD       "farm/cmd"           // downlink : watering duration

#define RELAY_PIN         5
#define RELAY_ON_LEVEL    HIGH               // HIGH = valve open (use LOW for active-low modules)
#define LED_PIN           8                  // onboard LED on SuperMini boards (active low)

#define MAX_WATER_SECONDS 3600UL             // safety cap on a single watering command (1 h)
// ===========================================================================

// ESP-NOW packet  ***  must match node1_sensor.ino exactly  ***
typedef struct __attribute__((packed)) {
  float   temperature;
  float   humidity;
  uint8_t validSamples;                      // 0 = every DHT read failed
} Packet;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

bool     relayOn      = false;               // is the valve open right now
uint32_t relayOffAtMs = 0;                   // millis() value at which it must close

volatile bool   rxPending = false;           // sensor data waiting for loop() to pick up
volatile Packet rxPacket;

uint32_t lastMqttTry = 0;
uint32_t lastWifiTry = 0;

void relayStop();

// ---------------------------------------------------------------------------
// RELAY
// ---------------------------------------------------------------------------

// Drive the relay pin and the onboard LED together
void relayWrite(bool on) {
  digitalWrite(RELAY_PIN, on ? RELAY_ON_LEVEL : !RELAY_ON_LEVEL);
  digitalWrite(LED_PIN, on ? LOW : HIGH);
}

// Tell the server whether the valve is open, and for how many seconds
void publishRelay(int on, long seconds) {
  char buf[32];
  snprintf(buf, sizeof(buf), "S,relay,%d,%ld", on, seconds);
  if (mqtt.connected()) mqtt.publish(TOPIC_DATA, buf);
  Serial.printf("[PUB] %s\n", buf);
}

// Open the valve and record the millis() value at which it must close.
// No delay() is used, so nothing blocks. Anything above MAX_WATER_SECONDS is
// clamped; a repeat command while watering simply restarts the countdown.
void relayStart(uint32_t seconds) {
  if (seconds == 0) { relayStop(); return; }

  if (seconds > MAX_WATER_SECONDS) {
    Serial.printf("[RELAY] %lu s exceeds limit -> clamped to %lu s\n",
                  (unsigned long)seconds, MAX_WATER_SECONDS);
    seconds = MAX_WATER_SECONDS;
  }

  relayOffAtMs = millis() + seconds * 1000UL;
  relayOn      = true;
  relayWrite(true);
  Serial.printf("[RELAY] ON for %lu s\n", (unsigned long)seconds);
  publishRelay(1, seconds);
}

// Close the valve. Safe to call repeatedly.
void relayStop() {
  if (!relayOn) return;
  relayOn = false;
  relayWrite(false);
  Serial.println("[RELAY] OFF");
  publishRelay(0, 0);
}

// ---------------------------------------------------------------------------
// ESP-NOW  -  receive only, never replies, so no peer registration is needed
// ---------------------------------------------------------------------------

// [callback] Node 1 sent data. This runs in the WiFi task, so MQTT calls are
// forbidden here - just stash the packet and raise a flag for handleSensorPacket().
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(Packet)) return;
  memcpy((void *)&rxPacket, data, sizeof(Packet));
  rxPending = true;
}

// Format the pending packet as CSV and publish it to the server
void handleSensorPacket() {
  Packet p;
  memcpy(&p, (const void *)&rxPacket, sizeof(Packet));
  rxPending = false;

  char buf[48];
  snprintf(buf, sizeof(buf), "D,%.2f,%.2f,%u",
           p.temperature, p.humidity, p.validSamples);
  Serial.printf("[RX ] %s\n", buf);

  if (mqtt.connected()) mqtt.publish(TOPIC_DATA, buf);
  else Serial.println("[RX ] MQTT not connected -> this reading is lost");
}

// ---------------------------------------------------------------------------
// MQTT / WIFI
// ---------------------------------------------------------------------------

// [callback] The server published a command to farm/cmd.
// The payload is a plain number of seconds: "120" = water 120 s, "0" = stop now.
void onMqttMessage(char *topic, byte *payload, unsigned int length) {
  char s[16];
  unsigned int n = length < sizeof(s) - 1 ? length : sizeof(s) - 1;
  memcpy(s, payload, n);
  s[n] = 0;
  Serial.printf("[CMD] \"%s\"\n", s);
  relayStart(strtoul(s, NULL, 10));
}

// Keep MQTT alive: pump mqtt.loop() while connected, otherwise retry every 5 s.
// On every successful connect it re-subscribes and registers an LWT so the
// broker announces our death if we vanish without saying goodbye.
void mqttTick() {
  if (mqtt.connected()) { mqtt.loop(); return; }

  if (WiFi.status() != WL_CONNECTED) return;
  if (millis() - lastMqttTry < 5000) return;
  lastMqttTry = millis();

  Serial.print("[MQTT] connecting... ");
  const char *lwt = "S,offline,0,0";
  bool ok = strlen(MQTT_USER)
            ? mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS, TOPIC_DATA, 0, false, lwt)
            : mqtt.connect(MQTT_CLIENT_ID, TOPIC_DATA, 0, false, lwt);

  if (ok) {
    Serial.println("OK");
    mqtt.subscribe(TOPIC_CMD);
    mqtt.publish(TOPIC_DATA, "S,online,0,0");
  } else {
    Serial.printf("failed rc=%d\n", mqtt.state());
  }
}

// Keep WiFi alive: retry the connection every 5 s while it is down
void wifiTick() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastWifiTry < 5000) return;
  lastWifiTry = millis();
  Serial.println("[WIFI] lost -> reconnecting");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

// ---------------------------------------------------------------------------

// Bring everything up in order: close the valve first -> WiFi -> ESP-NOW -> MQTT,
// then print the MAC and channel to copy into node1_sensor.ino.
void setup() {
  // Close the relay as early as possible. Writing the level before switching the
  // pin to output means it drives "closed" immediately, with no open glitch on boot.
  relayWrite(false);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  relayWrite(false);

  Serial.begin(115200);
  delay(300);
  Serial.println("\n\n=== Node 0 (Root) ===");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                      // with modem sleep on, ESP-NOW drops packets
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("[WIFI] connecting");         // give up after 20 s, wifiTick() keeps retrying
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (esp_now_init() != ESP_OK) {            // must always come after WiFi.mode()
    Serial.println("[ESPNOW] init failed -> restarting");
    delay(2000);
    ESP.restart();
  }
  esp_now_register_recv_cb(onEspNowRecv);

  String mac = WiFi.macAddress();            // reformat so it can be pasted directly
  mac.replace(":", ", 0x");
  Serial.println("\n=====================================================");
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf(" WiFi OK   IP = %s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println(" WiFi failed! Check SSID/PASS (it will keep retrying)");
  Serial.println(" Copy these two lines into node1_sensor.ino :");
  Serial.printf("   #define NODE0_MAC     { 0x%s }\n", mac.c_str());
  Serial.printf("   #define WIFI_CHANNEL  %d\n", WiFi.channel());
  Serial.println("=====================================================\n");

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
}

// Poll four things forever. Every one is non-blocking so nothing can hold up
// the valve shut-off.
void loop() {
  if (relayOn && (int32_t)(millis() - relayOffAtMs) >= 0) relayStop();   // time to close?

  wifiTick();
  mqttTick();
  if (rxPending) handleSensorPacket();

  delay(5);                                  // yield the CPU to the WiFi task (C3 is single-core)
}
