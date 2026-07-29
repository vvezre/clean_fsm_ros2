#include <gtest/gtest.h>

#include <cstdio>

#include "cleanbot_modeling/sqlite_model_repository.hpp"

namespace {

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
