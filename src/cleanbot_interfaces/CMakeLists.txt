cmake_minimum_required(VERSION 3.8)
project(cleanbot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/TaskSegment.msg"
  "msg/BrushCommand.msg"
  "msg/CommandExecutionStatus.msg"
  "msg/VehicleCommand.msg"
  "msg/HardwareStatus.msg"
  "msg/MaintenanceState.msg"
  "msg/RtkFix.msg"
  "msg/VehicleState.msg"
  "msg/TrackingDebug.msg"
  "msg/TrackingTarget.msg"
  "msg/TrackingStatus.msg"
  "msg/ConfigEntry.msg"
  "msg/ConfigStatus.msg"
  "msg/ConfigChanged.msg"
  "msg/ModelPoint.msg"
  "msg/ModelSubArea.msg"
  "msg/ModelConnector.msg"
  "msg/ModelGroup.msg"
  "msg/CleaningModel.msg"
  "msg/CleaningPlan.msg"
  "srv/EmergencyStop.srv"
  "srv/ManualControl.srv"
  "srv/SetMissionPause.srv"
  "srv/SetMaintenanceMode.srv"
  "srv/GetConfig.srv"
  "srv/SetConfig.srv"
  "srv/ManageCleaningModel.srv"
  "srv/SampleModelPoint.srv"
  "srv/GenerateCleaningPlan.srv"
  "srv/ExecuteModelPlan.srv"
  "action/ExecuteCleaning.action"
  "action/NavigateWaypoints.action"
  "action/Dock.action"
  "action/ReturnHome.action"
  "action/RecoverMission.action"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
