#include "DoorStateMachine.h"
#include <unity.h>

void test_initial_state_is_closed() {
  DoorStateMachine m;
  TEST_ASSERT_EQUAL_INT((int)DoorState::Closed, (int)m.current());
}
void test_transition_records_history_and_reason() {
  DoorStateMachine m;
  DoorState from = DoorState::Fault, to = DoorState::Fault;
  const char *reason = nullptr;
  m.setTransitionCallback([&](DoorState f, DoorState t, const char *r) {
    from = f;
    to = t;
    reason = r;
  });
  TEST_ASSERT_TRUE(m.transitionTo(DoorState::Opening, "sensor"));
  TEST_ASSERT_EQUAL_INT((int)DoorState::Closed, (int)m.previous());
  TEST_ASSERT_EQUAL_INT((int)DoorState::Opening, (int)m.current());
  TEST_ASSERT_EQUAL_INT((int)DoorState::Closed, (int)from);
  TEST_ASSERT_EQUAL_INT((int)DoorState::Opening, (int)to);
  TEST_ASSERT_EQUAL_STRING("sensor", reason);
}
void test_duplicate_transition_is_ignored() {
  DoorStateMachine m;
  int calls = 0;
  m.setTransitionCallback([&](DoorState, DoorState, const char *) { ++calls; });
  TEST_ASSERT_FALSE(m.transitionTo(DoorState::Closed, "duplicate"));
  TEST_ASSERT_EQUAL_INT(0, calls);
  TEST_ASSERT_EQUAL_INT((int)DoorState::Closed, (int)m.previous());
}
void test_state_machine_tracks_multiple_transitions() {
  DoorStateMachine m;
  int calls = 0;
  DoorState previous = DoorState::Closed;
  const DoorState states[]{DoorState::Opening, DoorState::Open,
                           DoorState::Closing, DoorState::Closed};
  m.setTransitionCallback(
      [&](DoorState from, DoorState to, const char *reason) {
        TEST_ASSERT_EQUAL_INT((int)previous, (int)from);
        TEST_ASSERT_EQUAL_INT((int)states[calls++], (int)to);
        TEST_ASSERT_EQUAL_STRING("sequence", reason);
      });
  for (DoorState next : states) {
    TEST_ASSERT_TRUE(m.transitionTo(next, "sequence"));
    TEST_ASSERT_EQUAL_INT((int)previous, (int)m.previous());
    previous = next;
  }
  TEST_ASSERT_EQUAL_INT(4, calls);
}
void test_transition_supports_null_callback_and_reason() {
  DoorStateMachine m;
  TEST_ASSERT_TRUE(m.transitionTo(DoorState::Opening));
  const char *reason = "set";
  m.setTransitionCallback(
      [&](DoorState, DoorState, const char *r) { reason = r; });
  TEST_ASSERT_TRUE(m.transitionTo(DoorState::Open));
  TEST_ASSERT_NULL(reason);
}
void setUp() {}
void tearDown() {}
int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_initial_state_is_closed);
  RUN_TEST(test_transition_records_history_and_reason);
  RUN_TEST(test_duplicate_transition_is_ignored);
  RUN_TEST(test_state_machine_tracks_multiple_transitions);
  RUN_TEST(test_transition_supports_null_callback_and_reason);
  return UNITY_END();
}
