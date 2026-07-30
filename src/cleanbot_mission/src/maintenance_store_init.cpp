#include <iostream>

#include "cleanbot_mission/maintenance_store.hpp"

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
