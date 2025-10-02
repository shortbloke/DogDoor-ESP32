#include "mqtt.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <array>
#include <cstdint>
#include "Secrets.h"
#include "DisplayHelpers.h"

// Serial debug print macro matches DoorController.cpp usage.
#ifdef SERIAL_PRINT_ENABLE
  #define SERIAL_PRINT(...) Serial.printf(__VA_ARGS__)
#else
  #define SERIAL_PRINT(...)
#endif

// ---------- Configuration ----------
static const char* MQTT_CLIENT_ID = "DogDoorESP32";

// Base topics (retain simple, HA discovery will point to these)
static const char* TOPIC_DOOR_STATE          = "dogdoor/door/state";
static const char* TOPIC_DOOR_STATE_ACTUAL   = "dogdoor/door/state_actual";
static const char* TOPIC_DISTANCE_INDOOR     = "dogdoor/distance/indoor";
static const char* TOPIC_DISTANCE_OUTDOOR    = "dogdoor/distance/outdoor";
static const char* TOPIC_LIMIT_BOTTOM        = "dogdoor/limit/bottom";
static const char* TOPIC_LIMIT_TOP           = "dogdoor/limit/top";
static const char* TOPIC_SENSOR_TRIGGER      = "dogdoor/sensor_trigger";
static const char* TOPIC_AVAILABILITY       = "dogdoor/status";  // LWT + availability
static constexpr const char* TOF_STATUS_PAYLOAD_OK = "OK";
static constexpr const char* TOF_STATUS_PAYLOAD_ERROR = "ERROR";

// Home Assistant discovery prefix & topics (component/node_id/object_id/config)
static const char* DISCOVERY_PREFIX          = "homeassistant";
static const char* NODE_ID                   = "dogdoor";

static const char* DISCOVERY_DOOR_CFG        = "homeassistant/binary_sensor/dogdoor/door/config";
static const char* DISCOVERY_DOOR_STATE_CFG  = "homeassistant/sensor/dogdoor/door_state/config";
static const char* DISCOVERY_DIST_IN_CFG     = "homeassistant/sensor/dogdoor/distance_indoor/config";
static const char* DISCOVERY_DIST_OUT_CFG    = "homeassistant/sensor/dogdoor/distance_outdoor/config";
static const char* DISCOVERY_LIMIT_BOTTOM_CFG = "homeassistant/binary_sensor/dogdoor/limit_bottom/config";
static const char* DISCOVERY_LIMIT_TOP_CFG    = "homeassistant/binary_sensor/dogdoor/limit_top/config";
static const char* DISCOVERY_SENSOR_TRIGGER_CFG = "homeassistant/sensor/dogdoor/sensor_trigger/config";

// ---------- Internal State ----------
static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);
static DoorTelemetryProvider *telemetry = nullptr;

static bool discoveryPublished = false;
static String lastDoorState = "";
static unsigned long lastDistancePublish = 0;
static uint32_t distancePublishIntervalMs = 30000; // default 30s
static uint8_t lastSensorTriggerPublished = 0;
static std::array<String, Config::numTOFSensors> tofStatusTopics;
static std::array<String, Config::numTOFSensors> tofStatusDiscoveryTopics;
static std::array<String, Config::numTOFSensors> tofStatusUniqueIds;
static std::array<String, Config::numTOFSensors> tofStatusFriendlyNames;
static std::array<int8_t, Config::numTOFSensors> tofStatusLatest;
static std::array<int8_t, Config::numTOFSensors> tofStatusPublished;
static bool tofTopicsInitialized = false;

// ---------- Helpers ----------
static bool publishRetained(const char* topic, const String& payload) {
  if (!mqttClient.publish(topic, payload.c_str(), true)) {
    SERIAL_PRINT("[MQTT] Publish failed (%s) len=%u\n", topic, payload.length());
    return false;
  }
  return true;
}

static String slugifySensorName(const char* name) {
  String slug = name ? name : "sensor";
  slug.toLowerCase();
  for (size_t i = 0; i < slug.length(); ++i) {
    char c = slug.charAt(i);
    bool isAlphaNum = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9');
    if (!isAlphaNum) {
      slug.setCharAt(i, '_');
    }
  }
  while (slug.indexOf("__") != -1) {
    slug.replace("__", "_");
  }
  while (slug.length() > 0 && slug.charAt(0) == '_') {
    slug.remove(0, 1);
  }
  while (slug.length() > 0 && slug.charAt(slug.length() - 1) == '_') {
    slug.remove(slug.length() - 1, 1);
  }
  if (slug.length() == 0) {
    slug = "sensor";
  }
  return slug;
}

static void ensureTofTopicsInitialized() {
  if (tofTopicsInitialized) {
    return;
  }
  for (size_t i = 0; i < Config::numTOFSensors; ++i) {
    const char* name = Config::sensorNames[i];
    String slug = slugifySensorName(name);
    if (slug.length() == 0) {
      slug = String("sensor_") + i;
    }
    tofStatusTopics[i] = String("dogdoor/tof/") + slug + "/status";
    tofStatusDiscoveryTopics[i] = String(DISCOVERY_PREFIX) + "/binary_sensor/" + NODE_ID + "/tof_" + slug + "_status/config";
    tofStatusUniqueIds[i] = String("dogdoor_tof_") + slug + "_status";
    tofStatusFriendlyNames[i] = name ? name : "Sensor";
  }
  tofTopicsInitialized = true;
}

static void resetTofStatusCaches() {
  tofStatusLatest.fill(-1);
  tofStatusPublished.fill(-1);
}

static const char* normalizeDoorState(const char* raw) {
  // Expecting "OPEN"/"CLOSED" ideally. Map anything else.
  if (!raw) return "UNKNOWN";
  String s = raw;
  s.toUpperCase();
  if (s.indexOf("OPEN") >= 0) return "OPEN";
  if (s.indexOf("CLOSE") >= 0 || s == "SHUT") return "CLOSED";
  return raw; // pass through if custom
}

void mqttPublishDoorState(const char* state) {
  if (!mqttClient.connected()) return;

  const char* normalized = normalizeDoorState(state);
  const char* raw = state ? state : "UNKNOWN";

  mqttClient.publish(TOPIC_DOOR_STATE, normalized, true);
  mqttClient.publish(TOPIC_DOOR_STATE_ACTUAL, raw, true);
}

static void mqttPublishFloat(const char* topic, float value) {
  if (!mqttClient.connected()) return;
  if (value < 0) return;
  char buf[16];
  dtostrf(value, 1, 2, buf);
  mqttClient.publish(topic, buf, true);
}

void mqttPublishDistanceIndoor(float cm)  { mqttPublishFloat(TOPIC_DISTANCE_INDOOR, cm); }
void mqttPublishDistanceOutdoor(float cm) { mqttPublishFloat(TOPIC_DISTANCE_OUTDOOR, cm); }

static void mqttPublishLimit(const char* topic, bool pressed) {
  if (!mqttClient.connected()) return;
  mqttClient.publish(topic, pressed ? "ON" : "OFF", true);
}

void mqttPublishLimitSwitchTop(bool pressed) {
  mqttPublishLimit(TOPIC_LIMIT_TOP, pressed);
}

void mqttPublishLimitSwitchBottom(bool pressed) {
  mqttPublishLimit(TOPIC_LIMIT_BOTTOM, pressed);
}

void mqttPublishSensorTrigger(uint8_t triggerId) {
  if (!mqttClient.connected()) return;
  const char* payload = "NONE";
  if (triggerId == 1) payload = "INDOOR";
  else if (triggerId == 2) payload = "OUTDOOR";
  mqttClient.publish(TOPIC_SENSOR_TRIGGER, payload, true);
}

void mqttPublishTOFSensorStatus(uint8_t sensorIndex, bool measurementOk) {
  ensureTofTopicsInitialized();
  if (sensorIndex >= Config::numTOFSensors) {
    return;
  }

  int8_t latest = measurementOk ? 1 : 0;
  tofStatusLatest[sensorIndex] = latest;

  if (!mqttClient.connected()) {
    return;
  }

  if (tofStatusPublished[sensorIndex] == latest) {
    return;
  }

  const char* payload = measurementOk ? TOF_STATUS_PAYLOAD_OK : TOF_STATUS_PAYLOAD_ERROR;
  if (mqttClient.publish(tofStatusTopics[sensorIndex].c_str(), payload, true)) {
    tofStatusPublished[sensorIndex] = latest;
  } else {
    SERIAL_PRINT("[MQTT] Publish failed (%s)\n", tofStatusTopics[sensorIndex].c_str());
  }
}

void mqttSetDistancePublishInterval(uint32_t ms) {
  distancePublishIntervalMs = ms;
}

// Home Assistant discovery payloads
static void publishDiscovery() {
  if (!mqttClient.connected()) return;

  ensureTofTopicsInitialized();

  String deviceObj =
    "{\"identifiers\":[\"dogdoor_esp32\"],"
    "\"name\":\"Dog Door\","
    "\"manufacturer\":\"Martin Rowan\","
    "\"model\":\"ESP32\","
    "\"sw_version\":\"1.0.0\"}";

  // Binary sensor (door)
  String doorPayload =
    String("{"
      "\"name\":\"Dog Door\","
      "\"unique_id\":\"dogdoor_door\","
      "\"device_class\":\"door\","
      "\"state_topic\":\"") + TOPIC_DOOR_STATE + "\","
      "\"payload_on\":\"OPEN\","
      "\"payload_off\":\"CLOSED\","
      "\"availability_topic\":\"" + String(TOPIC_AVAILABILITY) + "\","
      "\"payload_available\":\"online\","
      "\"payload_not_available\":\"offline\","
      "\"device\":" + deviceObj +
    "}";

  // Door state text sensor
  String doorStatePayload =
    String("{"
      "\"name\":\"Dog Door State\","
      "\"unique_id\":\"dogdoor_state\","
      "\"state_topic\":\"") + TOPIC_DOOR_STATE_ACTUAL + "\","
      "\"availability_topic\":\"" + String(TOPIC_AVAILABILITY) + "\","
      "\"payload_available\":\"online\","
      "\"payload_not_available\":\"offline\","
      "\"icon\":\"mdi:door\","
      "\"device\":" + deviceObj +
    "}";

  // Indoor distance sensor
  String distInPayload =
    String("{"
      "\"name\":\"Dog Door Distance Indoor\","
      "\"unique_id\":\"dogdoor_distance_indoor\","
      "\"device_class\":\"distance\","
      "\"state_class\":\"measurement\","
      "\"unit_of_measurement\":\"cm\","
      "\"state_topic\":\"") + TOPIC_DISTANCE_INDOOR + "\","
      "\"availability_topic\":\"" + String(TOPIC_AVAILABILITY) + "\","
      "\"payload_available\":\"online\","
      "\"payload_not_available\":\"offline\","
      "\"device\":" + deviceObj +
    "}";

  // Outdoor distance sensor
  String distOutPayload =
    String("{"
      "\"name\":\"Dog Door Distance Outdoor\","
      "\"unique_id\":\"dogdoor_distance_outdoor\","
      "\"device_class\":\"distance\","
      "\"state_class\":\"measurement\","
      "\"unit_of_measurement\":\"cm\","
      "\"state_topic\":\"") + TOPIC_DISTANCE_OUTDOOR + "\","
      "\"availability_topic\":\"" + String(TOPIC_AVAILABILITY) + "\","
      "\"payload_available\":\"online\","
      "\"payload_not_available\":\"offline\","
      "\"device\":" + deviceObj +
    "}";

  // Bottom limit switch sensor
  String limitBottomPayload =
    String("{"
      "\"name\":\"Dog Door Limit Bottom\","
      "\"unique_id\":\"dogdoor_limit_bottom\","
      "\"device_class\":\"door\","
      "\"state_topic\":\"") + TOPIC_LIMIT_BOTTOM + "\","
      "\"payload_on\":\"ON\","
      "\"payload_off\":\"OFF\","
      "\"availability_topic\":\"" + String(TOPIC_AVAILABILITY) + "\","
      "\"payload_available\":\"online\","
      "\"payload_not_available\":\"offline\","
      "\"device\":" + deviceObj +
    "}";

  // Top limit switch sensor
  String limitTopPayload =
    String("{"
      "\"name\":\"Dog Door Limit Top\","
      "\"unique_id\":\"dogdoor_limit_top\","
      "\"device_class\":\"door\","
      "\"state_topic\":\"") + TOPIC_LIMIT_TOP + "\","
      "\"payload_on\":\"ON\","
      "\"payload_off\":\"OFF\","
      "\"availability_topic\":\"" + String(TOPIC_AVAILABILITY) + "\","
      "\"payload_available\":\"online\","
      "\"payload_not_available\":\"offline\","
      "\"device\":" + deviceObj +
    "}";

  // Sensor trigger text sensor
  String sensorTriggerPayload =
    String("{"
      "\"name\":\"Dog Door Sensor Trigger\","
      "\"unique_id\":\"dogdoor_sensor_trigger\","
      "\"state_topic\":\"") + TOPIC_SENSOR_TRIGGER + "\","
      "\"availability_topic\":\"" + String(TOPIC_AVAILABILITY) + "\","
      "\"payload_available\":\"online\","
      "\"payload_not_available\":\"offline\","
      "\"icon\":\"mdi:arrow-decision\","
      "\"device\":" + deviceObj +
    "}";

  bool ok = true;
  ok &= publishRetained(DISCOVERY_DOOR_CFG, doorPayload);
  ok &= publishRetained(DISCOVERY_DOOR_STATE_CFG, doorStatePayload);
  ok &= publishRetained(DISCOVERY_DIST_IN_CFG, distInPayload);
  ok &= publishRetained(DISCOVERY_DIST_OUT_CFG, distOutPayload);
  ok &= publishRetained(DISCOVERY_LIMIT_BOTTOM_CFG, limitBottomPayload);
  ok &= publishRetained(DISCOVERY_LIMIT_TOP_CFG, limitTopPayload);
  ok &= publishRetained(DISCOVERY_SENSOR_TRIGGER_CFG, sensorTriggerPayload);

  for (size_t i = 0; i < Config::numTOFSensors; ++i) {
    String sensorName = String("Dog Door ") + tofStatusFriendlyNames[i] + " TOF Status";
    String payload =
      String("{") +
        "\"name\":\"" + sensorName + "\"," +
        "\"unique_id\":\"" + tofStatusUniqueIds[i] + "\"," +
        "\"device_class\":\"problem\"," +
        "\"state_topic\":\"" + tofStatusTopics[i] + "\"," +
        "\"payload_on\":\"" + TOF_STATUS_PAYLOAD_ERROR + "\"," +
        "\"payload_off\":\"" + TOF_STATUS_PAYLOAD_OK + "\"," +
        "\"availability_topic\":\"" + String(TOPIC_AVAILABILITY) + "\"," +
        "\"payload_available\":\"online\"," +
        "\"payload_not_available\":\"offline\"," +
        "\"device\":" + deviceObj +
      "}";
    ok &= publishRetained(tofStatusDiscoveryTopics[i].c_str(), payload);
  }

  if (ok) {
    discoveryPublished = true;
  }
}

static void publishAllDistances() {
  if (!telemetry) return;
  mqttPublishDistanceIndoor(telemetry->getDistanceIndoorCm());
  mqttPublishDistanceOutdoor(telemetry->getDistanceOutdoorCm());
}

static void mqttReconnect() {
  if (WiFi.status() != WL_CONNECTED) return;

  while (!mqttClient.connected()) {
    // Set LWT (availability) on connect attempt
    if (mqttClient.connect(
          MQTT_CLIENT_ID,
          MQTT_USER, MQTT_PASSWORD,
          TOPIC_AVAILABILITY, 0, true, "offline"
        )) {
      SERIAL_PRINT("[MQTT] Connected\n");
      // Mark online (retain)
      mqttClient.publish(TOPIC_AVAILABILITY, "online", true);
      DisplayHelpers::setMQTTConnected(true);

      if (!discoveryPublished) publishDiscovery();

      const char* st = telemetry ? telemetry->getDoorStateString() : nullptr;
      uint8_t triggerId = telemetry ? telemetry->getLastSensorTriggered() : 0;
      mqttPublishDoorState(st);
      publishAllDistances();
      if (telemetry) {
        mqttPublishLimitSwitchBottom(!telemetry->isLimitSwitchPressed(BottomLimitSwitch));
        mqttPublishLimitSwitchTop(!telemetry->isLimitSwitchPressed(TopLimitSwitch));
        mqttPublishSensorTrigger(triggerId);
      }
      for (size_t i = 0; i < Config::numTOFSensors; ++i) {
        if (tofStatusLatest[i] != -1) {
          tofStatusPublished[i] = -1;
          mqttPublishTOFSensorStatus(i, tofStatusLatest[i] == 1);
        }
      }
      lastDoorState = st ? st : "";
      lastSensorTriggerPublished = triggerId;
      lastDistancePublish = millis();
    } else {
      SERIAL_PRINT("[MQTT] Connect failed rc=%d\n", mqttClient.state());
      delay(2000);
    }
  }
}

// ---------- Public API ----------
void mqttSetup(DoorTelemetryProvider *provider) {
  ensureTofTopicsInitialized();
  resetTofStatusCaches();
  telemetry = provider;
  discoveryPublished = false;
  lastDoorState = "";
  lastSensorTriggerPublished = telemetry ? telemetry->getLastSensorTriggered() : 0;
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setBufferSize(512);
}

void mqttLoop() {
  if (!mqttClient.connected()) {
    DisplayHelpers::setMQTTConnected(false);
    mqttReconnect();
  }
  mqttClient.loop();

  // Door state change detection (polling)
  const char* current = telemetry ? telemetry->getDoorStateString() : nullptr;
  String cur = current ? current : "";
  if (cur != lastDoorState) {
    mqttPublishDoorState(cur.c_str());
    publishAllDistances(); // snapshot at transition
    lastDoorState = cur;
  }

  if (telemetry) {
    uint8_t triggerId = telemetry->getLastSensorTriggered();
    if (triggerId != lastSensorTriggerPublished) {
      mqttPublishSensorTrigger(triggerId);
      lastSensorTriggerPublished = triggerId;
    }
  }
  // Periodic distance publish
  unsigned long now = millis();
  if (now - lastDistancePublish >= distancePublishIntervalMs) {
    publishAllDistances();
    lastDistancePublish = now;
  }
}
