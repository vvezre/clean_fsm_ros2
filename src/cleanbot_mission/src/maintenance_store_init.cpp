/*
 * 文件作用：维护存储初始化程序：创建并校验持久化维护状态数据库。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include <iostream>

#include "cleanbot_mission/maintenance_store.hpp"

// 命令行入口：校验状态文件路径，并创建可供维护运行时使用的初始存储文件。
int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "usage: maintenance_store_init STATE_PATH\n";
    return 2;
  }

  cleanbot::mission::MaintenanceStore store(argv[1]);
  const auto result = store.initializeGenesis();
  if (result.code != cleanbot::mission::MaintenanceStoreCode::kOk) {
    std::cerr << "maintenance store initialization failed: "
              << result.message << '\n';
    return 1;
  }

  std::cout << result.message << '\n';
  return 0;
}
