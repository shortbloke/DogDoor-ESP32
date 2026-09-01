#include "DoorSafetyPolicy.h"
#include <unity.h>

void test_homing_completes_only_at_bottom_limit() {
  TEST_ASSERT_EQUAL_INT(
      (int)LimitSafetyAction::None,
      (int)limitSafetyAction(DoorState::Homing, false, false));
  TEST_ASSERT_EQUAL_INT((int)LimitSafetyAction::CompleteHoming,
                        (int)limitSafetyAction(DoorState::Homing, true, false));
}
void test_conflicting_limits_always_fault() {
  for (DoorState s : {DoorState::Homing, DoorState::Closed, DoorState::Opening,
                      DoorState::Open, DoorState::Closing, DoorState::Fault})
    TEST_ASSERT_EQUAL_INT((int)LimitSafetyAction::FaultConflictingLimits,
                          (int)limitSafetyAction(s, true, true));
}
void test_directional_limits_complete_motion() {
  TEST_ASSERT_EQUAL_INT(
      (int)LimitSafetyAction::CompleteOpening,
      (int)limitSafetyAction(DoorState::Opening, false, true));
  TEST_ASSERT_EQUAL_INT(
      (int)LimitSafetyAction::CompleteClosing,
      (int)limitSafetyAction(DoorState::Closing, true, false));
}
void test_non_directional_limits_do_not_complete_motion() {
  TEST_ASSERT_EQUAL_INT(
      (int)LimitSafetyAction::None,
      (int)limitSafetyAction(DoorState::Opening, true, false));
  TEST_ASSERT_EQUAL_INT(
      (int)LimitSafetyAction::None,
      (int)limitSafetyAction(DoorState::Closing, false, true));
  TEST_ASSERT_EQUAL_INT((int)LimitSafetyAction::None,
                        (int)limitSafetyAction(DoorState::Homing, false, true));
  for (DoorState s : {DoorState::Closed, DoorState::Open, DoorState::Fault}) {
    TEST_ASSERT_EQUAL_INT((int)LimitSafetyAction::None,
                          (int)limitSafetyAction(s, true, false));
    TEST_ASSERT_EQUAL_INT((int)LimitSafetyAction::None,
                          (int)limitSafetyAction(s, false, true));
  }
}
void test_tof_polling_policy_covers_all_states() {
  TEST_ASSERT_FALSE(shouldPollTofSensors(DoorState::Homing));
  TEST_ASSERT_FALSE(shouldPollTofSensors(DoorState::Opening));
  TEST_ASSERT_FALSE(shouldPollTofSensors(DoorState::Fault));
  TEST_ASSERT_TRUE(shouldPollTofSensors(DoorState::Closed));
  TEST_ASSERT_TRUE(shouldPollTofSensors(DoorState::Open));
  TEST_ASSERT_TRUE(shouldPollTofSensors(DoorState::Closing));
}
void test_door_request_respects_safety_and_overrides() {
  TEST_ASSERT_TRUE(
      shouldRequestDoorOpen(DoorState::Closed, true, false, false));
  TEST_ASSERT_FALSE(
      shouldRequestDoorOpen(DoorState::Closed, false, false, false));
  TEST_ASSERT_TRUE(
      shouldRequestDoorOpen(DoorState::Closing, false, true, false));
  TEST_ASSERT_FALSE(shouldRequestDoorOpen(DoorState::Open, true, true, true));
  TEST_ASSERT_FALSE(
      shouldRequestDoorOpen(DoorState::Homing, true, true, false));
}
void test_binary_state_is_closed_only_when_confirmed() {
  TEST_ASSERT_EQUAL_STRING("CLOSED", binaryDoorState(DoorState::Closed));
  for (DoorState s : {DoorState::Homing, DoorState::Opening, DoorState::Open,
                      DoorState::Closing, DoorState::Fault})
    TEST_ASSERT_EQUAL_STRING("OPEN", binaryDoorState(s));
}
void setUp() {}
void tearDown() {}
int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_homing_completes_only_at_bottom_limit);
  RUN_TEST(test_conflicting_limits_always_fault);
  RUN_TEST(test_directional_limits_complete_motion);
  RUN_TEST(test_non_directional_limits_do_not_complete_motion);
  RUN_TEST(test_tof_polling_policy_covers_all_states);
  RUN_TEST(test_door_request_respects_safety_and_overrides);
  RUN_TEST(test_binary_state_is_closed_only_when_confirmed);
  return UNITY_END();
}
