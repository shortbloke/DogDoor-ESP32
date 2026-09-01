#include "SensorQualification.h"
#include <unity.h>

void test_qualifies_after_required_readings_once() {
  SensorQualification s(3);
  auto r = s.update(true, 99, 100, 1000);
  TEST_ASSERT_FALSE(r.qualified);
  TEST_ASSERT_FALSE(r.triggered);
  TEST_ASSERT_FALSE(s.update(true, 99, 100, 1000).qualified);
  r = s.update(true, 99, 100, 1000);
  TEST_ASSERT_TRUE(r.qualified);
  TEST_ASSERT_TRUE(r.triggered);
  r = s.update(true, 99, 100, 1000);
  TEST_ASSERT_TRUE(r.qualified);
  TEST_ASSERT_FALSE(r.triggered);
}
void test_streak_resets_on_clear_or_invalid_reading() {
  SensorQualification s(2);
  s.update(true, 50, 100, 1000);
  auto r = s.update(true, 100, 100, 1000);
  TEST_ASSERT_FALSE(r.qualified);
  TEST_ASSERT_EQUAL_UINT8(0, s.consecutiveReadings());
  s.update(true, 50, 100, 1000);
  r = s.update(false, 50, 100, 1000);
  TEST_ASSERT_FALSE(r.qualified);
  TEST_ASSERT_EQUAL_UINT8(0, s.consecutiveReadings());
}
void test_maximum_distance_boundary_is_inclusive() {
  SensorQualification s(1);
  TEST_ASSERT_TRUE(s.update(true, 1000, 1001, 1000).triggered);
  s.reset();
  TEST_ASSERT_FALSE(s.update(true, 1001, 2000, 1000).qualified);
}
void test_instances_keep_independent_streaks() {
  SensorQualification a(2), b(2);
  a.update(true, 50, 100, 1000);
  TEST_ASSERT_FALSE(b.update(true, 50, 100, 1000).qualified);
  TEST_ASSERT_TRUE(b.update(true, 50, 100, 1000).triggered);
  TEST_ASSERT_FALSE(a.qualified());
  TEST_ASSERT_TRUE(a.update(true, 50, 100, 1000).triggered);
}
void test_zero_required_readings_is_treated_as_one() {
  SensorQualification s(0);
  TEST_ASSERT_TRUE(s.update(true, 50, 100, 1000).triggered);
}
void setUp() {}
void tearDown() {}
int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_qualifies_after_required_readings_once);
  RUN_TEST(test_streak_resets_on_clear_or_invalid_reading);
  RUN_TEST(test_maximum_distance_boundary_is_inclusive);
  RUN_TEST(test_instances_keep_independent_streaks);
  RUN_TEST(test_zero_required_readings_is_treated_as_one);
  return UNITY_END();
}
