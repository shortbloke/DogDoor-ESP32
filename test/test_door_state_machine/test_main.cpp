#include <unity.h>

#include "DoorStateMachine.h"

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

void setUp() {}
void tearDown() {}

int main(int, char **)
{
  UNITY_BEGIN();
  RUN_TEST(test_initial_state_is_closed);
  RUN_TEST(test_transition_records_previous_state_and_reason);
  RUN_TEST(test_duplicate_transition_is_ignored);
  return UNITY_END();
}
