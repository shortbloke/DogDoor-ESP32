#include <unity.h>

#include "DoorStateMachine.h"
#include "DoorSafetyPolicy.h"

void test_initial_state_is_closed()
{
  DoorStateMachine machine;
  TEST_ASSERT_EQUAL(static_cast<int>(DoorState::Closed), static_cast<int>(machine.current()));
}

void test_transition_records_previous_state_and_reason()
{
  DoorStateMachine machine;
  DoorState observedFrom = DoorState::Fault;
  DoorState observedTo = DoorState::Fault;
  const char *observedReason = nullptr;
  machine.setTransitionCallback([&](DoorState from, DoorState to, const char *reason) {
    observedFrom = from;
    observedTo = to;
    observedReason = reason;
  });

  TEST_ASSERT_TRUE(machine.transitionTo(DoorState::Opening, "sensor"));
  TEST_ASSERT_EQUAL(static_cast<int>(DoorState::Closed), static_cast<int>(machine.previous()));
  TEST_ASSERT_EQUAL(static_cast<int>(DoorState::Opening), static_cast<int>(machine.current()));
  TEST_ASSERT_EQUAL(static_cast<int>(DoorState::Closed), static_cast<int>(observedFrom));
  TEST_ASSERT_EQUAL(static_cast<int>(DoorState::Opening), static_cast<int>(observedTo));
  TEST_ASSERT_EQUAL_STRING("sensor", observedReason);
}

void test_duplicate_transition_is_ignored()
{
  DoorStateMachine machine;
  TEST_ASSERT_FALSE(machine.transitionTo(DoorState::Closed, "duplicate"));
}

void test_homing_completes_only_at_bottom_limit()
{
  TEST_ASSERT_EQUAL(static_cast<int>(LimitSafetyAction::None),
                    static_cast<int>(limitSafetyAction(DoorState::Homing, false, false)));
  TEST_ASSERT_EQUAL(static_cast<int>(LimitSafetyAction::CompleteHoming),
                    static_cast<int>(limitSafetyAction(DoorState::Homing, true, false)));
}

void test_conflicting_limits_always_fault()
{
  for (DoorState state : {DoorState::Homing, DoorState::Closed, DoorState::Opening,
                          DoorState::Open, DoorState::Closing, DoorState::Fault})
  {
    TEST_ASSERT_EQUAL(static_cast<int>(LimitSafetyAction::FaultConflictingLimits),
                      static_cast<int>(limitSafetyAction(state, true, true)));
  }
}

void test_directional_limit_completes_motion()
{
  TEST_ASSERT_EQUAL(static_cast<int>(LimitSafetyAction::CompleteOpening),
                    static_cast<int>(limitSafetyAction(DoorState::Opening, false, true)));
  TEST_ASSERT_EQUAL(static_cast<int>(LimitSafetyAction::CompleteClosing),
                    static_cast<int>(limitSafetyAction(DoorState::Closing, true, false)));
}

void test_tof_polling_prioritizes_motion_safety()
{
  TEST_ASSERT_FALSE(shouldPollTofSensors(DoorState::Homing));
  TEST_ASSERT_FALSE(shouldPollTofSensors(DoorState::Opening));
  TEST_ASSERT_FALSE(shouldPollTofSensors(DoorState::Fault));
  TEST_ASSERT_TRUE(shouldPollTofSensors(DoorState::Closing));
}

void test_binary_state_is_closed_only_when_confirmed()
{
  TEST_ASSERT_EQUAL_STRING("CLOSED", binaryDoorState(DoorState::Closed));
  TEST_ASSERT_EQUAL_STRING("OPEN", binaryDoorState(DoorState::Homing));
  TEST_ASSERT_EQUAL_STRING("OPEN", binaryDoorState(DoorState::Opening));
  TEST_ASSERT_EQUAL_STRING("OPEN", binaryDoorState(DoorState::Open));
  TEST_ASSERT_EQUAL_STRING("OPEN", binaryDoorState(DoorState::Closing));
  TEST_ASSERT_EQUAL_STRING("OPEN", binaryDoorState(DoorState::Fault));
}

void setUp() {}
void tearDown() {}

int main(int, char **)
{
  UNITY_BEGIN();
  RUN_TEST(test_initial_state_is_closed);
  RUN_TEST(test_transition_records_previous_state_and_reason);
  RUN_TEST(test_duplicate_transition_is_ignored);
  RUN_TEST(test_homing_completes_only_at_bottom_limit);
  RUN_TEST(test_conflicting_limits_always_fault);
  RUN_TEST(test_directional_limit_completes_motion);
  RUN_TEST(test_tof_polling_prioritizes_motion_safety);
  RUN_TEST(test_binary_state_is_closed_only_when_confirmed);
  return UNITY_END();
}
