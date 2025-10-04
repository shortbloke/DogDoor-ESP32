#pragma once

#include "Config.h"
#include <functional>

// Lightweight state machine wrapper that centralises transitions and allows
// observers to audit them.
class DoorStateMachine {
public:
  using TransitionCallback = std::function<void(DoorState from, DoorState to, const char *reason)>;

  DoorStateMachine() = default;

  DoorState current() const { return currentState; }
  DoorState previous() const { return previousState; }

  bool transitionTo(DoorState next, const char *reason = nullptr);
  void setTransitionCallback(TransitionCallback cb) { callback = std::move(cb); }

private:
  DoorState currentState = DoorState::Closed;
  DoorState previousState = DoorState::Closed;
  TransitionCallback callback;
};
