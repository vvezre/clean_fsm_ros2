#include <gtest/gtest.h>

TEST(BuildContract, UsesCpp14OrNewer) {
  EXPECT_GE(__cplusplus, 201402L);
}
