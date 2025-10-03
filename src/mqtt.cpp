#include "mqtt.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <array>
#include <cstdint>
#include <cstdio>
#include <limits>
#include "Secrets.h"
#include "DisplayHelpers.h"

// Serial debug print macro matches DoorController.cpp usage.
#ifdef SERIAL_PRINT_ENABLE
  #define SERIAL_PRINT(...) Serial.printf(__VA_ARGS__)
#else
  #define SERIAL_PRINT(...)
#endif

// ---------- Configuration ----------
constexpr char kMqttClientId[] = "DogDoorESP32";

// Base topics (retain simple, HA discovery will point to these)
constexpr char kTopicDoorState[] = "dogdoor/door/state";
constexpr char kTopicDoorStateActual[] = "dogdoor/door/state_actual";
constexpr char kTopicDistanceIndoor[] = "dogdoor/distance/indoor";
constexpr char kTopicDistanceOutdoor[] = "dogdoor/distance/outdoor";
constexpr char kTopicLimitBottom[] = "dogdoor/limit/bottom";
constexpr char kTopicLimitTop[] = "dogdoor/limit/top";
constexpr char kTopicSensorTrigger[] = "dogdoor/sensor_trigger";
constexpr char kTopicAvailability[] = "dogdoor/status";  // LWT + availability
constexpr char kTofStatusPayloadOk[] = "OK";
constexpr char kTofStatusPayloadError[] = "ERROR";

// Home Assistant discovery prefix & topics (component/node_id/object_id/config)
constexpr char kDiscoveryPrefix[] = "homeassistant";
constexpr char kNodeId[] = "dogdoor";

constexpr char kDiscoveryDoorConfig[] = "homeassistant/binary_sensor/dogdoor/door/config";
constexpr char kDiscoveryDoorStateConfig[] = "homeassistant/sensor/dogdoor/door_state/config";
constexpr char kDiscoveryDistanceIndoorConfig[] = "homeassistant/sensor/dogdoor/distance_indoor/config";
constexpr char kDiscoveryDistanceOutdoorConfig[] = "homeassistant/sensor/dogdoor/distance_outdoor/config";
constexpr char kDiscoveryLimitBottomConfig[] = "homeassistant/binary_sensor/dogdoor/limit_bottom/config";
constexpr char kDiscoveryLimitTopConfig[] = "homeassistant/binary_sensor/dogdoor/limit_top/config";
constexpr char kDiscoverySensorTriggerConfig[] = "homeassistant/sensor/dogdoor/sensor_trigger/config";

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
static std::array<String, Config::numTOFSensors> tofInitTopics;
static std::array<String, Config::numTOFSensors> tofInitDiscoveryTopics;
static std::array<String, Config::numTOFSensors> tofInitUniqueIds;
static std::array<uint32_t, Config::numTOFSensors> tofInitCounters;
static std::array<uint32_t, Config::numTOFSensors> tofInitPublished;
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
    tofStatusDiscoveryTopics[i] = String(kDiscoveryPrefix) + "/binary_sensor/" + kNodeId + "/tof_" + slug + "_status/config";
    tofStatusUniqueIds[i] = String("dogdoor_tof_") + slug + "_status";
    tofStatusFriendlyNames[i] = name ? name : "Sensor";
    tofInitTopics[i] = String("dogdoor/tof/") + slug + "/init_count";
    tofInitDiscoveryTopics[i] = String(kDiscoveryPrefix) + "/sensor/" + kNodeId + "/tof_" + slug + "_init/config";
    tofInitUniqueIds[i] = String("dogdoor_tof_") + slug + "_init";
  }
  tofTopicsInitialized = true;
}

static void resetTofStatusCaches() {
  tofStatusLatest.fill(-1);
  tofStatusPublished.fill(-1);
  tofInitCounters.fill(0);
  tofInitPublished.fill(std::numeric_limits<uint32_t>::max());
}

static void publishTofInitCount(size_t sensorIndex) {
  if (!mqttClient.connected()) {
    return;
  }
  if (sensorIndex >= Config::numTOFSensors) {
    return;
  }
  if (tofInitPublished[sensorIndex] == tofInitCounters[sensorIndex]) {
    return;
  }
  char buffer[12];
  snprintf(buffer, sizeof(buffer), "%lu", static_cast<unsigned long>(tofInitCounters[sensorIndex]));
  if (mqttClient.publish(tofInitTopics[sensorIndex].c_str(), buffer, true)) {
    tofInitPublished[sensorIndex] = tofInitCounters[sensorIndex];
  } else {
    SERIAL_PRINT("[MQTT] Publish failed (%s)\n", tofInitTopics[sensorIndex].c_str());
  }
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

  mqttClient.publish(kTopicDoorState, normalized, true);
  mqttClient.publish(kTopicDoorStateActual, raw, true);
}

static void mqttPublishFloat(const char* topic, float value) {
  if (!mqttClient.connected()) return;
  if (value < 0) return;
  char buf[16];
  dtostrf(value, 1, 2, buf);
  mqttClient.publish(topic, buf, true);
}

void mqttPublishDistanceIndoor(float cm)  { mqttPublishFloat(kTopicDistanceIndoor, cm); }
void mqttPublishDistanceOutdoor(float cm) { mqttPublishFloat(kTopicDistanceOutdoor, cm); }

static void mqttPublishLimit(const char* topic, bool pressed) {
  if (!mqttClient.connected()) return;
  mqttClient.publish(topic, pressed ? "ON" : "OFF", true);
}

void mqttPublishLimitSwitchTop(bool pressed) {
  mqttPublishLimit(kTopicLimitTop, pressed);
}

void mqttPublishLimitSwitchBottom(bool pressed) {
  mqttPublishLimit(kTopicLimitBottom, pressed);
}

void mqttPublishSensorTrigger(uint8_t triggerId) {
  if (!mqttClient.connected()) return;
  const char* payload = "NONE";
  if (triggerId == 1) payload = "INDOOR";
  else if (triggerId == 2) payload = "OUTDOOR";
  mqttClient.publish(kTopicSensorTrigger, payload, true);
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

  const char* payload = measurementOk ? kTofStatusPayloadOk : kTofStatusPayloadError;
  if (mqttClient.publish(tofStatusTopics[sensorIndex].c_str(), payload, true)) {
    tofStatusPublished[sensorIndex] = latest;
  } else {
    SERIAL_PRINT("[MQTT] Publish failed (%s)\n", tofStatusTopics[sensorIndex].c_str());
  }
}

void mqttSetDistancePublishInterval(uint32_t ms) {
  distancePublishIntervalMs = ms;
}

void mqttPublishTOFInit(uint8_t sensorIndex) {
  ensureTofTopicsInitialized();
  if (sensorIndex >= Config::numTOFSensors) {
    return;
  }
  ++tofInitCounters[sensorIndex];
  publishTofInitCount(sensorIndex);
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
      "\"state_topic\":\"") + kTopicDoorState + "\","
      "\"payload_on\":\"OPEN\","
      "\"payload_off\":\"CLOSED\","
      "\"availability_topic\":\"" + String(kTopicAvailability) + "\","
      "\"payload_available\":\"online\","
      "\"payload_not_available\":\"offline\","
      "\"device\":" + deviceObj +
    "}";

  // Door state text sensor
  String doorStatePayload =
    String("{"
      "\"name\":\"Dog Door State\","
      "\"unique_id\":\"dogdoor_state\","
      "\"state_topic\":\"") + kTopicDoorStateActual + "\","
      "\"availability_topic\":\"" + String(kTopicAvailability) + "\","
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
      "\"state_topic\":\"") + kTopicDistanceIndoor + "\","
      "\"availability_topic\":\"" + String(kTopicAvailability) + "\","
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
      "\"state_topic\":\"") + kTopicDistanceOutdoor + "\","
      "\"availability_topic\":\"" + String(kTopicAvailability) + "\","
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
      "\"state_topic\":\"") + kTopicLimitBottom + "\","
      "\"payload_on\":\"ON\","
      "\"payload_off\":\"OFF\","
      "\"availability_topic\":\"" + String(kTopicAvailability) + "\","
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
      "\"state_topic\":\"") + kTopicLimitTop + "\","
      "\"payload_on\":\"ON\","
      "\"payload_off\":\"OFF\","
      "\"availability_topic\":\"" + String(kTopicAvailability) + "\","
      "\"payload_available\":\"online\","
      "\"payload_not_available\":\"offline\","
      "\"device\":" + deviceObj +
    "}";

  // Sensor trigger text sensor
  String sensorTriggerPayload =
    String("{"
      "\"name\":\"Dog Door Sensor Trigger\","
      "\"unique_id\":\"dogdoor_sensor_trigger\","
      "\"state_topic\":\"") + kTopicSensorTrigger + "\","
      "\"availability_topic\":\"" + String(kTopicAvailability) + "\","
      "\"payload_available\":\"online\","
      "\"payload_not_available\":\"offline\","
      "\"icon\":\"mdi:arrow-decision\","
      "\"device\":" + deviceObj +
    "}";

  bool ok = true;
  ok &= publishRetained(kDiscoveryDoorConfig, doorPayload);
  ok &= publishRetained(kDiscoveryDoorStateConfig, doorStatePayload);
  ok &= publishRetained(kDiscoveryDistanceIndoorConfig, distInPayload);
  ok &= publishRetained(kDiscoveryDistanceOutdoorConfig, distOutPayload);
  ok &= publishRetained(kDiscoveryLimitBottomConfig, limitBottomPayload);
  ok &= publishRetained(kDiscoveryLimitTopConfig, limitTopPayload);
  ok &= publishRetained(kDiscoverySensorTriggerConfig, sensorTriggerPayload);

  for (size_t i = 0; i < Config::numTOFSensors; ++i) {
    String sensorName = String("Dog Door ") + tofStatusFriendlyNames[i] + " TOF Status";
    String payload =
      String("{") +
        "\"name\":\"" + sensorName + "\"," +
        "\"unique_id\":\"" + tofStatusUniqueIds[i] + "\"," +
        "\"device_class\":\"problem\"," +
        "\"state_topic\":\"" + tofStatusTopics[i] + "\"," +
        "\"payload_on\":\"" + kTofStatusPayloadError + "\"," +
        "\"payload_off\":\"" + kTofStatusPayloadOk + "\"," +
        "\"availability_topic\":\"" + String(kTopicAvailability) + "\"," +
        "\"payload_available\":\"online\"," +
        "\"payload_not_available\":\"offline\"," +
        "\"device\":" + deviceObj +
      "}";
    ok &= publishRetained(tofStatusDiscoveryTopics[i].c_str(), payload);

    String initPayload =
      String("{") +
        "\"name\":\"Dog Door " + tofStatusFriendlyNames[i] + " TOF Init Count\"," +
        "\"unique_id\":\"" + tofInitUniqueIds[i] + "\"," +
        "\"state_topic\":\"" + tofInitTopics[i] + "\"," +
        "\"availability_topic\":\"" + String(kTopicAvailability) + "\"," +
        "\"payload_available\":\"online\"," +
        "\"payload_not_available\":\"offline\"," +
        "\"unit_of_measurement\":\"count\"," +
        "\"icon\":\"mdi:counter\"," +
        "\"device\":" + deviceObj +
      "}";
    ok &= publishRetained(tofInitDiscoveryTopics[i].c_str(), initPayload);
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
          kMqttClientId,
          MQTT_USER, MQTT_PASSWORD,
          kTopicAvailability, 0, true, "offline"
        )) {
      SERIAL_PRINT("[MQTT] Connected\n");
      // Mark online (retain)
      mqttClient.publish(kTopicAvailability, "online", true);
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
        publishTofInitCount(i);
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
