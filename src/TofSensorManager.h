#pragma once

#include <VL53L0X.h>
#include <array>
#include <cstdint>
#include <functional>
#include "Config.h"

class DisplayService;

// Encapsulates VL53L0X management (initialisation, polling, fault handling)
// so high-level logic can reason about sensor outcomes instead of I2C details.
class TofSensorManager {
public:
  static constexpr size_t kSensorCount = TofSensorConfig::count;

  struct UpdateResult {
    bool anySensorQualified = false;
    bool triggerEvent = false;
    uint8_t triggerSensorId = 0; // 0 = none, 1 = indoor, 2 = outdoor
    std::array<uint16_t, kSensorCount> rangesMm{};
  };

  using StatusPublisher = std::function<void(uint8_t sensorIndex, bool ok, bool reportInitEvent)>;

  explicit TofSensorManager(DisplayService *displayService = nullptr);

  void setDisplayService(DisplayService *service);
  void setStatusPublisher(StatusPublisher publisher);

  bool begin(bool allowRetry = true);
  UpdateResult update();

  float distanceCm(size_t index) const;
  uint8_t lastTriggerId() const { return lastSensorTriggered; }

  void requestReinitialisation();

  const std::array<uint16_t, kSensorCount>& rangesMm() const { return ranges; }
  const std::array<bool, kSensorCount>& readyFlags() const { return sensorReady; }
  const std::array<bool, kSensorCount>& measurementFlags() const { return measurementOk; }

private:
  bool initialiseSensors(bool isReinit);
  void publishSensorStatus(uint8_t sensorIndex, bool ok, bool reportInitEvent);

  DisplayService *display = nullptr;
  StatusPublisher statusPublisher;

  std::array<VL53L0X, kSensorCount> sensors;
  std::array<bool, kSensorCount> sensorReady{{false, false}};
  std::array<bool, kSensorCount> sensorBelowThreshold{{false, false}};
  std::array<uint16_t, kSensorCount> ranges{{0, 0}};
  std::array<unsigned long, kSensorCount> sensorStatusSuppressUntil{{0, 0}};
  std::array<uint8_t, kSensorCount> sensorBelowStreak{{0, 0}};
  std::array<bool, kSensorCount> measurementOk{{false, false}};

  unsigned long lastSensorInitMs = 0;
  unsigned long nextReinitAttemptMs = 0;
  bool sensorInitNeeded = false;
  uint8_t lastSensorTriggered = 0;
};
