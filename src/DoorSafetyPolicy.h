#pragma once

#include "Config.h"

enum class LimitSafetyAction
{
  None,
  FaultConflictingLimits,
  CompleteHoming,
  CompleteOpening,
  CompleteClosing
};

constexpr LimitSafetyAction limitSafetyAction(DoorState state, bool bottomPressed, bool topPressed)
{
  if (bottomPressed && topPressed)
    return LimitSafetyAction::FaultConflictingLimits;
  if (state == DoorState::Homing && bottomPressed)
    return LimitSafetyAction::CompleteHoming;
  if (state == DoorState::Opening && topPressed)
    return LimitSafetyAction::CompleteOpening;
  if (state == DoorState::Closing && bottomPressed)
    return LimitSafetyAction::CompleteClosing;
  return LimitSafetyAction::None;
}

// Closing continues to poll because obstruction detection must still reopen it.
constexpr bool shouldPollTofSensors(DoorState state)
{
  return state == DoorState::Closed || state == DoorState::Open || state == DoorState::Closing;
}

// Only a positively established closed state is safe to report as closed.
constexpr const char *binaryDoorState(DoorState state)
{
  return state == DoorState::Closed ? "CLOSED" : "OPEN";
}
