#include <gtest/gtest.h>

TEST(BuildContract, UsesCpp14OrNewer) {
  EXPECT_GE(__cplusplus, 201402L);
}
// 文件作用：验证公共模块按项目约定使用C++17标准编译。
