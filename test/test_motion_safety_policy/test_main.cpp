#include "MotionSafetyPolicy.h"
#include <unity.h>

void test_duration_elapsed_handles_boundaries_and_wraparound() {
  TEST_ASSERT_FALSE(durationElapsed(1099, 1000, 100));
  TEST_ASSERT_TRUE(durationElapsed(1100, 1000, 100));
  TEST_ASSERT_TRUE(durationElapsed(3, UINT32_MAX - 5, 9));
}
void test_motion_bounds_check_time_and_both_directions() {
  TEST_ASSERT_FALSE(motionBoundsExceeded(1099, 1000, 100, 50, 0, 100));
  TEST_ASSERT_TRUE(motionBoundsExceeded(1100, 1000, 100, 50, 0, 100));
  TEST_ASSERT_TRUE(motionBoundsExceeded(900, 1000, 100, 50, 0, 100));
  TEST_ASSERT_TRUE(motionBoundsExceeded(1000, 1000, 100, 100, 0, 100));
}
void setUp() {}
void tearDown() {}
int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_duration_elapsed_handles_boundaries_and_wraparound);
  RUN_TEST(test_motion_bounds_check_time_and_both_directions);
  return UNITY_END();
}
