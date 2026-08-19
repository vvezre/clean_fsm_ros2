// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include <gtest/gtest.h>

#include <cstdio>

#include "cleanbot_modeling/sqlite_model_repository.hpp"

namespace {

// 测试目的：验证 SqliteModelRepository.CreatesSchema 场景的行为、状态变化和边界条件。
TEST(SqliteModelRepository, CreatesSchema) {
  const std::string path = "test-modeling.db";
  std::remove(path.c_str());
  cleanbot::modeling::SqliteModelRepository repository(path);

  const auto status = repository.open_and_initialize();

  EXPECT_TRUE(status.healthy);
  EXPECT_EQ(status.schema_version, 3u);
  repository.close();
  std::remove(path.c_str());
}

}  // namespace
