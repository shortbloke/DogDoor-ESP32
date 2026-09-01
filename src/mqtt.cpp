#include "mqtt.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <array>
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <limits>
#include <pgmspace.h>
#include "Secrets.h"
#include "DisplayService.h"
#include "DoorSafetyPolicy.h"
#include "Version.h"

// Serial debug print macro matches DoorController.cpp usage.
#ifdef SERIAL_PRINT_ENABLE
  #define SERIAL_PRINT(...) Serial.printf(__VA_ARGS__)
#else
  #define SERIAL_PRINT(...)
#endif

// ---------- Configuration ----------
constexpr char kMqttClientId[] = "PetDoorESP32";

// Base topics (retain simple, HA discovery will point to these)
constexpr char kTopicDoorState[] = "pet_door/door/state";
constexpr char kTopicDoorStateActual[] = "pet_door/door/state_actual";
constexpr char kTopicDistanceIndoor[] = "pet_door/distance/indoor";
constexpr char kTopicDistanceOutdoor[] = "pet_door/distance/outdoor";
constexpr char kTopicLimitBottom[] = "pet_door/limit/bottom";
constexpr char kTopicLimitTop[] = "pet_door/limit/top";
constexpr char kTopicSensorTrigger[] = "pet_door/sensor_trigger";
constexpr std::array<const char *, TofSensorConfig::count> kTopicSensorTriggerCounts{
  "pet_door/sensor/indoor_count", "pet_door/sensor/outdoor_count"};
constexpr char kTopicAvailability[] = "pet_door/status";  // LWT + availability
constexpr char kTopicDoorOpenCount[] = "pet_door/door/open_count";
constexpr char kTofStatusPayloadOk[] = "OK";
constexpr char kTofStatusPayloadError[] = "ERROR";

// Home Assistant discovery prefix & topics (component/node_id/object_id/config)
constexpr char kDiscoveryPrefix[] = "homeassistant";
constexpr char kNodeId[] = "pet_door";

constexpr char kDiscoveryDoorConfig[] = "homeassistant/binary_sensor/pet_door/door/config";
constexpr char kDiscoveryDoorStateConfig[] = "homeassistant/sensor/pet_door/door_state/config";
constexpr char kDiscoveryDistanceIndoorConfig[] = "homeassistant/sensor/pet_door/distance_indoor/config";
constexpr char kDiscoveryDistanceOutdoorConfig[] = "homeassistant/sensor/pet_door/distance_outdoor/config";
constexpr char kDiscoveryLimitBottomConfig[] = "homeassistant/binary_sensor/pet_door/limit_bottom/config";
constexpr char kDiscoveryLimitTopConfig[] = "homeassistant/binary_sensor/pet_door/limit_top/config";
constexpr char kDiscoverySensorTriggerConfig[] = "homeassistant/sensor/pet_door/sensor_trigger/config";
constexpr std::array<const char *, TofSensorConfig::count> kDiscoverySensorTriggerCountConfigs{
  "homeassistant/sensor/pet_door/indoor_trigger_count/config",
  "homeassistant/sensor/pet_door/outdoor_trigger_count/config"};
constexpr char kDiscoveryDoorOpenCountConfig[] = "homeassistant/sensor/pet_door/door_open_count/config";

// ---------- Internal State ----------
static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);
static DoorTelemetryProvider *telemetry = nullptr;
static DisplayService *display = nullptr;

static bool discoveryPublished = false;
static bool legacyDiscoveryCleared = false;
static DoorState lastDoorState = DoorState::Closed;
static bool lastDoorStateValid = false;
static char lastDoorStateLabel[64] = {0};
static unsigned long lastDistancePublish = 0;
static uint8_t lastSensorTriggerPublished = 0;
static uint32_t lastDoorOpenCount = 0;
static bool doorOpenCountValid = false;
static std::array<uint32_t, TofSensorConfig::count> lastSensorTriggerCounts{{0, 0}};
static std::array<bool, TofSensorConfig::count> sensorTriggerCountValid{{false, false}};
static std::array<String, TofSensorConfig::count> tofStatusTopics;
static std::array<String, TofSensorConfig::count> tofStatusDiscoveryTopics;
static std::array<String, TofSensorConfig::count> tofStatusUniqueIds;
static std::array<String, TofSensorConfig::count> tofStatusFriendlyNames;
static std::array<int8_t, TofSensorConfig::count> tofStatusLatest;
static std::array<int8_t, TofSensorConfig::count> tofStatusPublished;
static std::array<String, TofSensorConfig::count> tofInitTopics;
static std::array<String, TofSensorConfig::count> tofInitDiscoveryTopics;
static std::array<String, TofSensorConfig::count> tofInitUniqueIds;
static std::array<uint32_t, TofSensorConfig::count> tofInitCounters;
static std::array<uint32_t, TofSensorConfig::count> tofInitPublished;
static bool tofTopicsInitialized = false;
static constexpr uint32_t kDistancePublishIntervalMs = 10000; // default 10s
static unsigned long nextReconnectAttemptMs = 0;
static uint8_t reconnectAttemptStep = 0;
static constexpr unsigned long kReconnectBackoffBaseMs = 2000;
static constexpr unsigned long kReconnectBackoffMaxMs = 60000;

// Longest discovery payload is just over 450 bytes once the shared device
// metadata is injected, so give ourselves plenty of headroom.
static constexpr size_t kDiscoveryPayloadBufferSize = 512;
static const char kDeviceObjectTemplate[] PROGMEM =
  "{\"identifiers\":[\"pet_door_esp32\"],"
  "\"name\":\"Pet Door\","
  "\"manufacturer\":\"Martin Rowan\","
  "\"model\":\"ESP32\","
  "\"sw_version\":\"%s\"}";
static constexpr size_t kDeviceObjectBufferSize = 192;
static constexpr size_t kDeviceVersionBufferSize = 96;
static char deviceObjectJson[kDeviceObjectBufferSize];
static bool deviceObjectJsonInitialised = false;
static char deviceVersionBuffer[kDeviceVersionBufferSize];
static bool deviceVersionInitialised = false;

static const char kDoorTemplate[] PROGMEM =
  "{\"name\":\"Pet Door\",\"unique_id\":\"pet_door_door\",\"device_class\":\"door\","
  "\"state_topic\":\"%s\",\"payload_on\":\"OPEN\",\"payload_off\":\"CLOSED\","
  "\"availability_topic\":\"%s\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\","
  "\"device\":%s}";

static const char kDoorStateTemplate[] PROGMEM =
  "{\"name\":\"Pet Door State\",\"unique_id\":\"pet_door_state\",\"state_topic\":\"%s\","
  "\"availability_topic\":\"%s\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\","
  "\"icon\":\"mdi:door\",\"device\":%s}";

static const char kDistanceTemplate[] PROGMEM =
  "{\"name\":\"Pet Door Distance %s\",\"unique_id\":\"pet_door_distance_%s\",\"device_class\":\"distance\","
  "\"state_class\":\"measurement\",\"unit_of_measurement\":\"cm\",\"state_topic\":\"%s\",\"availability_topic\":\"%s\","
  "\"payload_available\":\"online\",\"payload_not_available\":\"offline\",\"device\":%s}";

static const char kLimitTemplate[] PROGMEM =
  "{\"name\":\"Pet Door Limit %s\",\"unique_id\":\"pet_door_limit_%s\",\"device_class\":\"door\","
  "\"state_topic\":\"%s\",\"payload_on\":\"ON\",\"payload_off\":\"OFF\",\"availability_topic\":\"%s\","
  "\"payload_available\":\"online\",\"payload_not_available\":\"offline\",\"device\":%s}";

static const char kSensorTriggerTemplate[] PROGMEM =
  "{\"name\":\"Pet Door Sensor Trigger\",\"unique_id\":\"pet_door_sensor_trigger\",\"state_topic\":\"%s\","
  "\"availability_topic\":\"%s\","
  "\"payload_available\":\"online\",\"payload_not_available\":\"offline\",\"icon\":\"mdi:arrow-decision\",\"device\":%s}";

static const char kDoorOpenCountTemplate[] PROGMEM =
  "{\"name\":\"Pet Door Open Count\",\"unique_id\":\"pet_door_open_count\",\"state_topic\":\"%s\","
  "\"availability_topic\":\"%s\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\","
  "\"state_class\":\"total_increasing\",\"unit_of_measurement\":\"count\",\"icon\":\"mdi:counter\",\"device\":%s}";

static const char kSensorTriggerCountTemplate[] PROGMEM =
  "{\"name\":\"Pet Door %s Trigger Count\",\"unique_id\":\"pet_door_%s_trigger_count\",\"state_topic\":\"%s\","
  "\"availability_topic\":\"%s\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\","
  "\"state_class\":\"total_increasing\",\"unit_of_measurement\":\"count\",\"icon\":\"mdi:counter\",\"device\":%s}";

static const char kTofStatusTemplate[] PROGMEM =
  "{\"name\":\"Pet Door %s TOF Status\",\"unique_id\":\"%s\",\"device_class\":\"problem\",\"state_topic\":\"%s\","
  "\"payload_on\":\"%s\",\"payload_off\":\"%s\",\"availability_topic\":\"%s\",\"payload_available\":\"online\","
  "\"payload_not_available\":\"offline\",\"device\":%s}";

static const char kTofInitTemplate[] PROGMEM =
  "{\"name\":\"Pet Door %s TOF Init Count\",\"unique_id\":\"%s\",\"state_topic\":\"%s\",\"availability_topic\":\"%s\","
  "\"payload_available\":\"online\",\"payload_not_available\":\"offline\",\"unit_of_measurement\":\"count\",\"icon\":\"mdi:counter\",\"device\":%s}";

struct DiscoveryPayloadCache {
  bool initialised = false;
  char door[kDiscoveryPayloadBufferSize];
  char doorState[kDiscoveryPayloadBufferSize];
  char distanceIndoor[kDiscoveryPayloadBufferSize];
  char distanceOutdoor[kDiscoveryPayloadBufferSize];
  char limitBottom[kDiscoveryPayloadBufferSize];
  char limitTop[kDiscoveryPayloadBufferSize];
  char sensorTrigger[kDiscoveryPayloadBufferSize];
  char doorOpenCount[kDiscoveryPayloadBufferSize];
  std::array<std::array<char, kDiscoveryPayloadBufferSize>, TofSensorConfig::count> sensorTriggerCounts{};
  std::array<std::array<char, kDiscoveryPayloadBufferSize>, TofSensorConfig::count> tofStatus{};
  std::array<std::array<char, kDiscoveryPayloadBufferSize>, TofSensorConfig::count> tofInit{};
};

static DiscoveryPayloadCache discoveryPayloads;
static DiscoveryPayloadCache publishedPayloads;
static bool publishedPayloadsValid = false;

static const char *deviceVersionString() {
  if (!deviceVersionInitialised) {
    String version(AppVersion::kVersionWithDate);
    if (AppVersion::kDirty) {
      version += F(" dirty");
    }
    if (version.length() >= kDeviceVersionBufferSize) {
      SERIAL_PRINT("[MQTT] Version string truncated for device info\n");
      version = version.substring(0, kDeviceVersionBufferSize - 1);
    }
    version.toCharArray(deviceVersionBuffer, kDeviceVersionBufferSize);
    deviceVersionInitialised = true;
  }
  return deviceVersionBuffer;
}

static const char *deviceObjectJsonString() {
  if (!deviceObjectJsonInitialised) {
    const int written = snprintf_P(
      deviceObjectJson,
      sizeof(deviceObjectJson),
      kDeviceObjectTemplate,
      deviceVersionString());
    if (written < 0 || static_cast<size_t>(written) >= sizeof(deviceObjectJson)) {
      SERIAL_PRINT("[MQTT] Device object JSON truncated\n");
      deviceObjectJson[sizeof(deviceObjectJson) - 1] = '\0';
    }
    deviceObjectJsonInitialised = true;
  }
  return deviceObjectJson;
}

static void ensureTofTopicsInitialized();

static void formatDiscoveryJson(char* buffer, size_t len, PGM_P fmt, ...) {
  if (!buffer || len == 0) {
    return;
  }
  va_list args;
  va_start(args, fmt);
  int written = vsnprintf_P(buffer, len, fmt, args);
  va_end(args);
  if (written < 0 || static_cast<size_t>(written) >= len) {
    SERIAL_PRINT("[MQTT] Discovery payload truncated\n");
    buffer[len - 1] = '\0';
  }
}

static void ensureDiscoveryPayloads() {
  ensureTofTopicsInitialized();
  if (discoveryPayloads.initialised) {
    return;
  }

  const char *deviceJson = deviceObjectJsonString();

  formatDiscoveryJson(
    discoveryPayloads.door,
    sizeof(discoveryPayloads.door),
    kDoorTemplate,
    kTopicDoorState,
    kTopicAvailability,
    deviceJson);

  formatDiscoveryJson(
    discoveryPayloads.doorState,
    sizeof(discoveryPayloads.doorState),
    kDoorStateTemplate,
    kTopicDoorStateActual,
    kTopicAvailability,
    deviceJson);

  formatDiscoveryJson(
    discoveryPayloads.distanceIndoor,
    sizeof(discoveryPayloads.distanceIndoor),
    kDistanceTemplate,
    "Indoor",
    "indoor",
    kTopicDistanceIndoor,
    kTopicAvailability,
    deviceJson);

  formatDiscoveryJson(
    discoveryPayloads.distanceOutdoor,
    sizeof(discoveryPayloads.distanceOutdoor),
    kDistanceTemplate,
    "Outdoor",
    "outdoor",
    kTopicDistanceOutdoor,
    kTopicAvailability,
    deviceJson);

  formatDiscoveryJson(
    discoveryPayloads.limitBottom,
    sizeof(discoveryPayloads.limitBottom),
    kLimitTemplate,
    "Bottom",
    "bottom",
    kTopicLimitBottom,
    kTopicAvailability,
    deviceJson);

  formatDiscoveryJson(
    discoveryPayloads.limitTop,
    sizeof(discoveryPayloads.limitTop),
    kLimitTemplate,
    "Top",
    "top",
    kTopicLimitTop,
    kTopicAvailability,
    deviceJson);

  formatDiscoveryJson(
    discoveryPayloads.sensorTrigger,
    sizeof(discoveryPayloads.sensorTrigger),
    kSensorTriggerTemplate,
    kTopicSensorTrigger,
    kTopicAvailability,
    deviceJson);

  formatDiscoveryJson(
    discoveryPayloads.doorOpenCount,
    sizeof(discoveryPayloads.doorOpenCount),
    kDoorOpenCountTemplate,
    kTopicDoorOpenCount,
    kTopicAvailability,
    deviceJson);

  for (size_t i = 0; i < Config.tof.count; ++i) {
    const char *friendly = Config.tof.names[i];
    const char *slug = i == 0 ? "indoor" : "outdoor";
    formatDiscoveryJson(
      discoveryPayloads.sensorTriggerCounts[i].data(),
      discoveryPayloads.sensorTriggerCounts[i].size(),
      kSensorTriggerCountTemplate,
      friendly,
      slug,
      kTopicSensorTriggerCounts[i],
      kTopicAvailability,
      deviceJson);
  }

  for (size_t i = 0; i < Config.tof.count; ++i) {
    const char* friendly = tofStatusFriendlyNames[i].c_str();
    formatDiscoveryJson(
      discoveryPayloads.tofStatus[i].data(),
      discoveryPayloads.tofStatus[i].size(),
      kTofStatusTemplate,
      friendly,
      tofStatusUniqueIds[i].c_str(),
      tofStatusTopics[i].c_str(),
      kTofStatusPayloadError,
      kTofStatusPayloadOk,
      kTopicAvailability,
      deviceJson);

    formatDiscoveryJson(
      discoveryPayloads.tofInit[i].data(),
      discoveryPayloads.tofInit[i].size(),
      kTofInitTemplate,
      friendly,
      tofInitUniqueIds[i].c_str(),
      tofInitTopics[i].c_str(),
      kTopicAvailability,
      deviceJson);
  }

  discoveryPayloads.initialised = true;
}

// ---------- Helpers ----------
static bool publishRetained(const char* topic, const char* payload) {
  if (!mqttClient.connected()) {
    return false;
  }
  if (!mqttClient.publish(topic, payload, true)) {
    SERIAL_PRINT("[MQTT] Publish failed (%s) len=%u\n", topic, static_cast<unsigned>(std::strlen(payload)));
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
  for (size_t i = 0; i < Config.tof.count; ++i) {
    const char* name = Config.tof.names[i];
    String slug = slugifySensorName(name);
    if (slug.length() == 0) {
      slug = String("sensor_") + i;
    }
    tofStatusTopics[i] = String("pet_door/tof/") + slug + "/status";
    tofStatusDiscoveryTopics[i] = String(kDiscoveryPrefix) + "/binary_sensor/" + kNodeId + "/tof_" + slug + "_status/config";
    tofStatusUniqueIds[i] = String("pet_door_tof_") + slug + "_status";
    tofStatusFriendlyNames[i] = name ? name : "Sensor";
    tofInitTopics[i] = String("pet_door/tof/") + slug + "/init_count";
    tofInitDiscoveryTopics[i] = String(kDiscoveryPrefix) + "/sensor/" + kNodeId + "/tof_" + slug + "_init/config";
    tofInitUniqueIds[i] = String("pet_door_tof_") + slug + "_init";
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
  if (sensorIndex >= Config.tof.count) {
    return;
  }
  if (tofInitPublished[sensorIndex] == tofInitCounters[sensorIndex]) {
    return;
  }
  char buffer[12];
  snprintf(buffer, sizeof(buffer), "%lu", static_cast<unsigned long>(tofInitCounters[sensorIndex]));
  if (publishRetained(tofInitTopics[sensorIndex].c_str(), buffer)) {
    tofInitPublished[sensorIndex] = tofInitCounters[sensorIndex];
  } else {
    SERIAL_PRINT("[MQTT] Publish failed (%s)\n", tofInitTopics[sensorIndex].c_str());
  }
}

void mqttPublishDoorState(DoorState state, const char *label) {
  const char *raw = label ? label : "UNKNOWN";

  publishRetained(kTopicDoorState, binaryDoorState(state));
  publishRetained(kTopicDoorStateActual, raw);
}

static void mqttPublishFloat(const char* topic, float value) {
  if (value < 0) return;
  char buf[16];
  dtostrf(value, 1, 2, buf);
  publishRetained(topic, buf);
}

void mqttPublishDistanceIndoor(float cm)  { mqttPublishFloat(kTopicDistanceIndoor, cm); }
void mqttPublishDistanceOutdoor(float cm) { mqttPublishFloat(kTopicDistanceOutdoor, cm); }

static void mqttPublishLimit(const char* topic, bool pressed) {
  publishRetained(topic, pressed ? "ON" : "OFF");
}

void mqttPublishLimitSwitchTop(bool pressed) {
  mqttPublishLimit(kTopicLimitTop, pressed);
}

void mqttPublishLimitSwitchBottom(bool pressed) {
  mqttPublishLimit(kTopicLimitBottom, pressed);
}

void mqttPublishSensorTrigger(uint8_t triggerId) {
  const char* payload = "NONE";
  if (triggerId == 1) payload = "INDOOR";
  else if (triggerId == 2) payload = "OUTDOOR";
  publishRetained(kTopicSensorTrigger, payload);
}

void mqttPublishSensorTriggerCount(size_t sensorIndex, uint32_t count) {
  if (sensorIndex >= kTopicSensorTriggerCounts.size()) {
    return;
  }
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%lu", static_cast<unsigned long>(count));
  publishRetained(kTopicSensorTriggerCounts[sensorIndex], buffer);
}

void mqttPublishDoorOpenCount(uint32_t count) {
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%lu", static_cast<unsigned long>(count));
  publishRetained(kTopicDoorOpenCount, buffer);
}

void mqttPublishTOFSensorStatus(uint8_t sensorIndex, bool measurementOk) {
  ensureTofTopicsInitialized();
  if (sensorIndex >= Config.tof.count) {
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
  if (publishRetained(tofStatusTopics[sensorIndex].c_str(), payload)) {
    tofStatusPublished[sensorIndex] = latest;
  } else {
    SERIAL_PRINT("[MQTT] Publish failed (%s)\n", tofStatusTopics[sensorIndex].c_str());
  }
}

void mqttPublishTOFInit(uint8_t sensorIndex) {
  ensureTofTopicsInitialized();
  if (sensorIndex >= Config.tof.count) {
    return;
  }
  ++tofInitCounters[sensorIndex];
  publishTofInitCount(sensorIndex);
}

// Home Assistant discovery payloads
static void publishDiscovery() {
  if (!mqttClient.connected()) return;

  ensureDiscoveryPayloads();

  bool ok = true;
  auto publishIfChanged = [&](const char *topic, const char *payload, char *publishedBuffer) {
    if (!publishedPayloadsValid || std::strcmp(publishedBuffer, payload) != 0) {
      if (publishRetained(topic, payload)) {
        std::strncpy(publishedBuffer, payload, kDiscoveryPayloadBufferSize - 1);
        publishedBuffer[kDiscoveryPayloadBufferSize - 1] = '\0';
        return true;
      }
      return false;
    }
    return true;
  };

  ok &= publishIfChanged(kDiscoveryDoorConfig, discoveryPayloads.door, publishedPayloads.door);
  ok &= publishIfChanged(kDiscoveryDoorStateConfig, discoveryPayloads.doorState, publishedPayloads.doorState);
  ok &= publishIfChanged(kDiscoveryDistanceIndoorConfig, discoveryPayloads.distanceIndoor, publishedPayloads.distanceIndoor);
  ok &= publishIfChanged(kDiscoveryDistanceOutdoorConfig, discoveryPayloads.distanceOutdoor, publishedPayloads.distanceOutdoor);
  ok &= publishIfChanged(kDiscoveryLimitBottomConfig, discoveryPayloads.limitBottom, publishedPayloads.limitBottom);
  ok &= publishIfChanged(kDiscoveryLimitTopConfig, discoveryPayloads.limitTop, publishedPayloads.limitTop);
  ok &= publishIfChanged(kDiscoverySensorTriggerConfig, discoveryPayloads.sensorTrigger, publishedPayloads.sensorTrigger);
  ok &= publishIfChanged(kDiscoveryDoorOpenCountConfig, discoveryPayloads.doorOpenCount, publishedPayloads.doorOpenCount);

  for (size_t i = 0; i < Config.tof.count; ++i) {
    ok &= publishIfChanged(kDiscoverySensorTriggerCountConfigs[i], discoveryPayloads.sensorTriggerCounts[i].data(), publishedPayloads.sensorTriggerCounts[i].data());
    ok &= publishIfChanged(tofStatusDiscoveryTopics[i].c_str(), discoveryPayloads.tofStatus[i].data(), publishedPayloads.tofStatus[i].data());
    ok &= publishIfChanged(tofInitDiscoveryTopics[i].c_str(), discoveryPayloads.tofInit[i].data(), publishedPayloads.tofInit[i].data());
  }

  if (ok) {
    publishedPayloadsValid = true;
    discoveryPublished = true;
  }
}

static bool clearLegacyDiscovery() {
  static constexpr const char *topics[] = {
    "homeassistant/binary_sensor/dogdoor/door/config",
    "homeassistant/sensor/dogdoor/door_state/config",
    "homeassistant/sensor/dogdoor/distance_indoor/config",
    "homeassistant/sensor/dogdoor/distance_outdoor/config",
    "homeassistant/binary_sensor/dogdoor/limit_bottom/config",
    "homeassistant/binary_sensor/dogdoor/limit_top/config",
    "homeassistant/sensor/dogdoor/sensor_trigger/config",
    "homeassistant/sensor/dogdoor/indoor_trigger_count/config",
    "homeassistant/sensor/dogdoor/outdoor_trigger_count/config",
    "homeassistant/sensor/dogdoor/door_open_count/config",
    "homeassistant/binary_sensor/dogdoor/tof_indoor_status/config",
    "homeassistant/binary_sensor/dogdoor/tof_outdoor_status/config",
    "homeassistant/sensor/dogdoor/tof_indoor_init/config",
    "homeassistant/sensor/dogdoor/tof_outdoor_init/config"
  };

  bool cleared = true;
  for (const char *topic : topics) {
    cleared &= publishRetained(topic, "");
  }
  return cleared;
}

static void publishAllDistances() {
  if (!telemetry) return;
  mqttPublishDistanceIndoor(telemetry->getDistanceIndoorCm());
  mqttPublishDistanceOutdoor(telemetry->getDistanceOutdoorCm());
}

static void mqttReconnect() {
  if (mqttClient.connected()) {
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    nextReconnectAttemptMs = millis();
    reconnectAttemptStep = 0;
    return;
  }

  unsigned long now = millis();
  if (static_cast<long>(now - nextReconnectAttemptMs) < 0) {
    return;
  }

  // Set LWT (availability) on connect attempt
  if (mqttClient.connect(
        kMqttClientId,
        MQTT_USER, MQTT_PASSWORD,
        kTopicAvailability, 0, true, "offline"
      )) {
    SERIAL_PRINT("[MQTT] Connected\n");
    // Mark online (retain)
    publishRetained(kTopicAvailability, "online");
    if (display) {
      display->setMQTTConnected(true);
    }

    if (!legacyDiscoveryCleared) {
      legacyDiscoveryCleared = clearLegacyDiscovery();
    }
    if (!discoveryPublished) {
      publishDiscovery();
    }

    const char *label = telemetry ? telemetry->getDoorStateString() : nullptr;
    const DoorState state = telemetry ? telemetry->getDoorState() : DoorState::Fault;
    uint8_t triggerId = telemetry ? telemetry->getLastSensorTriggered() : 0;
    mqttPublishDoorState(state, label);
    publishAllDistances();
    if (telemetry) {
      mqttPublishLimitSwitchBottom(telemetry->isLimitSwitchPressed(BottomLimitSwitch));
      mqttPublishLimitSwitchTop(telemetry->isLimitSwitchPressed(TopLimitSwitch));
      mqttPublishSensorTrigger(triggerId);
      mqttPublishDoorOpenCount(telemetry->getDoorOpenCount());
      for (size_t i = 0; i < Config.tof.count; ++i) {
        mqttPublishSensorTriggerCount(i, telemetry->getSensorTriggerCount(i));
      }
    }
    for (size_t i = 0; i < Config.tof.count; ++i) {
      if (tofStatusLatest[i] != -1) {
        tofStatusPublished[i] = -1;
        mqttPublishTOFSensorStatus(i, tofStatusLatest[i] == 1);
      }
      publishTofInitCount(i);
    }
    lastDoorState = state;
    lastDoorStateValid = true;
    strncpy(lastDoorStateLabel, label ? label : "", sizeof(lastDoorStateLabel) - 1);
    lastDoorStateLabel[sizeof(lastDoorStateLabel) - 1] = '\0';
    lastSensorTriggerPublished = triggerId;
    lastDistancePublish = millis();
    if (telemetry) {
      lastDoorOpenCount = telemetry->getDoorOpenCount();
      doorOpenCountValid = true;
      for (size_t i = 0; i < Config.tof.count; ++i) {
        lastSensorTriggerCounts[i] = telemetry->getSensorTriggerCount(i);
        sensorTriggerCountValid[i] = true;
      }
    } else {
      lastDoorOpenCount = 0;
      doorOpenCountValid = false;
      sensorTriggerCountValid.fill(false);
    }
    reconnectAttemptStep = 0;
    nextReconnectAttemptMs = now;
  } else {
    SERIAL_PRINT("[MQTT] Connect failed rc=%d\n", mqttClient.state());
    unsigned long delayMs = kReconnectBackoffBaseMs;
    if (reconnectAttemptStep > 0) {
      unsigned long candidate = kReconnectBackoffBaseMs << reconnectAttemptStep;
      delayMs = (candidate > kReconnectBackoffMaxMs) ? kReconnectBackoffMaxMs : candidate;
    }
    if (delayMs > kReconnectBackoffMaxMs) {
      delayMs = kReconnectBackoffMaxMs;
    }
    if (reconnectAttemptStep < 5) {
      ++reconnectAttemptStep;
    }
    nextReconnectAttemptMs = now + delayMs;
  }
}

// ---------- Public API ----------
void mqttSetup(DoorTelemetryProvider *provider, DisplayService *displayService) {
  ensureTofTopicsInitialized();
  resetTofStatusCaches();
  discoveryPayloads.initialised = false;
  telemetry = provider;
  display = displayService;
  discoveryPublished = false;
  legacyDiscoveryCleared = false;
  lastDoorStateValid = false;
  lastDoorStateLabel[0] = '\0';
  lastSensorTriggerPublished = telemetry ? telemetry->getLastSensorTriggered() : 0;
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setBufferSize(1024);
  reconnectAttemptStep = 0;
  nextReconnectAttemptMs = 0;
  doorOpenCountValid = false;
  sensorTriggerCountValid.fill(false);
  lastDoorOpenCount = telemetry ? telemetry->getDoorOpenCount() : 0;
}

void mqttLoop() {
  if (!mqttClient.connected()) {
    if (display) {
      display->setMQTTConnected(false);
    }
    mqttReconnect();
  }
  mqttClient.loop();

  // Door state change detection (polling)
  const char *currentLabel = telemetry ? telemetry->getDoorStateString() : "";
  const DoorState currentState = telemetry ? telemetry->getDoorState() : DoorState::Fault;
  if (!lastDoorStateValid || currentState != lastDoorState ||
      strncmp(currentLabel, lastDoorStateLabel, sizeof(lastDoorStateLabel)) != 0) {
    mqttPublishDoorState(currentState, currentLabel);
    publishAllDistances(); // snapshot at transition
    lastDoorState = currentState;
    lastDoorStateValid = true;
    strncpy(lastDoorStateLabel, currentLabel, sizeof(lastDoorStateLabel) - 1);
    lastDoorStateLabel[sizeof(lastDoorStateLabel) - 1] = '\0';
  }

  if (telemetry) {
    uint8_t triggerId = telemetry->getLastSensorTriggered();
    if (triggerId != lastSensorTriggerPublished) {
      mqttPublishSensorTrigger(triggerId);
      lastSensorTriggerPublished = triggerId;
    }
    uint32_t openCount = telemetry->getDoorOpenCount();
    if (!doorOpenCountValid || openCount != lastDoorOpenCount) {
      mqttPublishDoorOpenCount(openCount);
      lastDoorOpenCount = openCount;
      doorOpenCountValid = true;
    }
    for (size_t i = 0; i < Config.tof.count; ++i) {
      const uint32_t count = telemetry->getSensorTriggerCount(i);
      if (!sensorTriggerCountValid[i] || count != lastSensorTriggerCounts[i]) {
        mqttPublishSensorTriggerCount(i, count);
        lastSensorTriggerCounts[i] = count;
        sensorTriggerCountValid[i] = true;
      }
    }
  }
  // Periodic distance publish
  unsigned long now = millis();
  if (now - lastDistancePublish >= kDistancePublishIntervalMs) {
    publishAllDistances();
    lastDistancePublish = now;
  }
}
