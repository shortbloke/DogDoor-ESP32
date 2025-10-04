#include "DoorStateMachine.h"

bool DoorStateMachine::transitionTo(DoorState next, const char *reason)
{
  if (next == currentState)
  {
    return false;
  }
  DoorState from = currentState;
  previousState = currentState;
  currentState = next;
  if (callback)
  {
    callback(from, next, reason);
  }
  return true;
}
