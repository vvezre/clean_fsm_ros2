#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "cleanbot_common/qos_profiles.hpp"
#include "cleanbot_config/config_client.hpp"
#include "cleanbot_interfaces/action/execute_cleaning.hpp"
#include "cleanbot_interfaces/msg/cleaning_model.hpp"
#include "cleanbot_interfaces/msg/cleaning_plan.hpp"
#include "cleanbot_interfaces/msg/hardware_status.hpp"
#include "cleanbot_interfaces/msg/model_connector.hpp"
#include "cleanbot_interfaces/msg/model_group.hpp"
#include "cleanbot_interfaces/msg/model_point.hpp"
#include "cleanbot_interfaces/msg/model_sub_area.hpp"
#include "cleanbot_interfaces/msg/rtk_fix.hpp"
#include "cleanbot_interfaces/msg/task_segment.hpp"
#include "cleanbot_interfaces/srv/execute_model_plan.hpp"
#include "cleanbot_interfaces/srv/generate_cleaning_plan.hpp"
#include "cleanbot_interfaces/srv/manage_cleaning_model.hpp"
#include "cleanbot_interfaces/srv/sample_model_point.hpp"
#include "cleanbot_modeling/geometry.hpp"
#include "cleanbot_modeling/goal_submission.hpp"
#include "cleanbot_modeling/point_sampler.hpp"
#include "cleanbot_modeling/region_recognizer.hpp"
#include "cleanbot_modeling/sqlite_model_repository.hpp"
#include "cleanbot_modeling/task_plan_builder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace cleanbot {
namespace modeling {

class ModelingManagerNode final : public rclcpp::Node {
 public:
  using ExecuteCleaning = cleanbot_interfaces::action::ExecuteCleaning;
  using ManageCleaningModel =
      cleanbot_interfaces::srv::ManageCleaningModel;
  using SampleModelPoint = cleanbot_interfaces::srv::SampleModelPoint;
  using GenerateCleaningPlan =
      cleanbot_interfaces::srv::GenerateCleaningPlan;
  using ExecuteModelPlan = cleanbot_interfaces::srv::ExecuteModelPlan;

  ModelingManagerNode() : Node("modeling_manager_node") {
    io_group_ = create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);
    // Service callbacks may wait for RTK samples or an Action goal response.
    // Serialize them and keep Action callbacks in a separate group so a
    // blocked service can never consume every executor thread it depends on.
    service_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    action_group_ = create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = io_group_;
    rtk_subscription_ =
        create_subscription<cleanbot_interfaces::msg::RtkFix>(
            "/rtk/fix",
            common::rtk_fix_qos(),
            std::bind(
                &ModelingManagerNode::onRtkFix,
                this,
                std::placeholders::_1),
            subscription_options);
    hardware_subscription_ =
        create_subscription<cleanbot_interfaces::msg::HardwareStatus>(
            "/hardware/status",
            common::hardware_status_qos(),
            std::bind(
                &ModelingManagerNode::onHardwareStatus,
                this,
                std::placeholders::_1),
            subscription_options);

    manage_service_ = create_service<ManageCleaningModel>(
        "/modeling/manage",
        std::bind(
            &ModelingManagerNode::onManage,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_group_);
    sample_service_ = create_service<SampleModelPoint>(
        "/modeling/sample_point",
        std::bind(
            &ModelingManagerNode::onSamplePoint,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_group_);
    generate_service_ = create_service<GenerateCleaningPlan>(
        "/modeling/generate_plan",
        std::bind(
            &ModelingManagerNode::onGeneratePlan,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_group_);
    execute_service_ = create_service<ExecuteModelPlan>(
        "/modeling/execute_plan",
        std::bind(
            &ModelingManagerNode::onExecutePlan,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_group_);

    execute_client_ =
        rclcpp_action::create_client<ExecuteCleaning>(
            this,
            "/mission/execute_cleaning",
            action_group_);

    config_client_ = std::make_unique<config::ConfigClient>(
        this,
        std::vector<std::string>{
            "modeling.database_path",
            "modeling.brush_width_cm",
            "modeling.minimum_overlap_cm",
            "modeling.sample_count",
            "modeling.maximum_sample_radius_m",
            "modeling.recognition.duplicate_tolerance_cm",
            "modeling.recognition.minimum_area_cm2",
            "modeling.recognition.assist_turn_threshold_deg",
            "modeling.recognition.minimum_connector_length_cm",
            "modeling.recognition.maximum_connector_endpoint_distance_cm",
            "modeling.recognition.unordered_auto_confirm_confidence",
            "motion.base_forward_speed",
        },
        false,
        [this](
            const config::ConfigSnapshot& snapshot,
            const bool initial) {
          configure(snapshot, initial);
        });
  }

 private:
  struct BufferedSample {
    std::uint64_t sequence{0u};
    RtkSample sample;
  };

  static cleanbot_interfaces::msg::ModelPoint toRosPoint(
      const ModelPoint& point) {
    cleanbot_interfaces::msg::ModelPoint output;
    output.id = point.id;
    output.sequence = static_cast<std::uint32_t>(point.sequence);
    output.lat = point.lat;
    output.lon = point.lon;
    output.x_cm = point.x_cm;
    output.y_cm = point.y_cm;
    output.heading_deg = point.heading_deg;
    output.capture_type = point.capture_type;
    output.role = point.role;
    output.roles = point.roles;
    output.sample_count =
        static_cast<std::uint32_t>(point.sample_count);
    output.sample_radius_m = point.sample_radius_m;
    output.rtk_quality = point.fix_quality;
    output.gga_age_sec = point.gga_age_sec;
    output.source = point.source;
    return output;
  }

  static cleanbot_interfaces::msg::CleaningModel toRosModel(
      const CleaningModel& model) {
    cleanbot_interfaces::msg::CleaningModel output;
    output.id = model.id;
    output.name = model.name;
    output.version = model.version;
    output.status = model.status;
    output.origin_lat = model.origin_lat;
    output.origin_lon = model.origin_lon;
    output.origin_valid = model.origin_valid;
    output.recognition_confirmed = model.recognition_confirmed;
    output.preview_confirmed = model.preview_confirmed;
    for (const auto& group : model.groups) {
      cleanbot_interfaces::msg::ModelGroup group_message;
      group_message.id = group.id;
      group_message.name = group.name;
      group_message.area_number = group.area_number;
      group_message.sweep_mode = group.sweep_mode;
      group_message.sweep_angle_deg = group.sweep_angle_deg;
      group_message.recognition_status = group.recognition_status;
      group_message.recognition_message = group.recognition_message;
      group_message.recognition_confidence = group.recognition_confidence;
      for (const auto& point : group.points) {
        group_message.points.push_back(toRosPoint(point));
      }
      for (const auto& area : group.sub_areas) {
        cleanbot_interfaces::msg::ModelSubArea area_message;
        area_message.id = area.id;
        area_message.name = area.name;
        area_message.point_ids = area.point_ids;
        area_message.confirmed = area.confirmed;
        group_message.sub_areas.push_back(area_message);
      }
      for (const auto& connector : group.connectors) {
        cleanbot_interfaces::msg::ModelConnector connector_message;
        connector_message.id = connector.id;
        connector_message.type = connector.type;
        connector_message.start_point_id = connector.start_point_id;
        connector_message.end_point_id = connector.end_point_id;
        connector_message.from_sub_area_id = connector.from_sub_area_id;
        connector_message.to_sub_area_id = connector.to_sub_area_id;
        connector_message.length_cm = connector.length_cm;
        connector_message.confirmed = connector.confirmed;
        group_message.connectors.push_back(connector_message);
      }
      output.groups.push_back(group_message);
    }
    return output;
  }

  static cleanbot_interfaces::msg::TaskSegment toRosSegment(
      const PlanSegment& segment) {
    cleanbot_interfaces::msg::TaskSegment output;
    output.index = static_cast<std::uint32_t>(segment.index);
    output.id = segment.id;
    output.segment_type = segment.segment_type;
    output.group_id = segment.group_id;
    output.sub_area_id = segment.sub_area_id;
    output.source_lane_id = segment.source_lane_id;
    output.start_lat = segment.start_lat;
    output.start_lon = segment.start_lon;
    output.end_lat = segment.end_lat;
    output.end_lon = segment.end_lon;
    output.heading_deg = segment.heading_deg;
    output.turn_angle_deg = segment.turn_angle_deg;
    output.speed = segment.speed;
    output.mode = segment.mode;
    output.turn_back_length = 0.0;
    output.back_length = 0.0;
    return output;
  }

  static cleanbot_interfaces::msg::CleaningPlan toRosPlan(
      const CleaningPlan& plan) {
    cleanbot_interfaces::msg::CleaningPlan output;
    output.id = plan.id;
    output.model_id = plan.model_id;
    output.model_version = plan.model_version;
    output.plan_hash = plan.plan_hash;
    output.generated_at = plan.generated_at;
    output.brush_width_cm = plan.brush_width_cm;
    output.minimum_overlap_cm = plan.minimum_overlap_cm;
    output.actual_overlap_cm = plan.actual_overlap_cm;
    output.cleaning_lane_count =
        static_cast<std::uint32_t>(plan.cleaning_lane_count);
    output.transfer_segment_count =
        static_cast<std::uint32_t>(plan.transfer_segment_count);
    output.total_length_cm = plan.total_length_cm;
    for (const auto& segment : plan.segments) {
      output.segments.push_back(toRosSegment(segment));
    }
    return output;
  }

  std::string makeId(const std::string& prefix) {
    const auto sequence = ++id_sequence_;
    return prefix + "-" + std::to_string(now().nanoseconds()) +
        "-" + std::to_string(sequence);
  }

  void configure(
      const config::ConfigSnapshot& snapshot,
      const bool initial) {
    const auto database_path =
        snapshot.get_string("modeling.database_path");
    const double brush_width =
        snapshot.get_double("modeling.brush_width_cm");
    const double minimum_overlap =
        snapshot.get_double("modeling.minimum_overlap_cm");
    const auto sample_count = static_cast<std::size_t>(
        snapshot.get_integer("modeling.sample_count"));
    const double maximum_radius =
        snapshot.get_double("modeling.maximum_sample_radius_m");
    RecognitionOptions recognition_options;
    recognition_options.duplicate_tolerance_cm = snapshot.get_double(
        "modeling.recognition.duplicate_tolerance_cm");
    recognition_options.minimum_area_cm2 = snapshot.get_double(
        "modeling.recognition.minimum_area_cm2");
    recognition_options.assist_turn_threshold_deg = snapshot.get_double(
        "modeling.recognition.assist_turn_threshold_deg");
    recognition_options.minimum_connector_length_cm = snapshot.get_double(
        "modeling.recognition.minimum_connector_length_cm");
    recognition_options.maximum_connector_endpoint_distance_cm = snapshot.get_double(
        "modeling.recognition.maximum_connector_endpoint_distance_cm");
    recognition_options.unordered_auto_confirm_confidence = snapshot.get_double(
        "modeling.recognition.unordered_auto_confirm_confidence");
    const auto base_speed = static_cast<std::int32_t>(
        snapshot.get_integer("motion.base_forward_speed"));

    std::lock_guard<std::mutex> repository_lock(repository_mutex_);
    auto next_repository =
        std::make_unique<SqliteModelRepository>(database_path);
    const auto status = next_repository->open_and_initialize();
    if (!status.healthy) {
      configured_.store(false);
      RCLCPP_ERROR(
          get_logger(),
          "%s: %s",
          status.code.c_str(),
          status.message.c_str());
      return;
    }
    repository_ = std::move(next_repository);
    brush_width_cm_ = brush_width;
    minimum_overlap_cm_ = minimum_overlap;
    sample_count_ = sample_count;
    maximum_sample_radius_m_ = maximum_radius;
    recognition_options_ = recognition_options;
    base_forward_speed_ = base_speed;
    configured_.store(true);
    RCLCPP_INFO(
        get_logger(),
        "modeling database ready%s",
        initial ? "" : " after configuration refresh");
  }

  void onHardwareStatus(
      const cleanbot_interfaces::msg::HardwareStatus::SharedPtr message) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_hardware_ = *message;
      hardware_received_ = true;
    }
    sample_condition_.notify_all();
  }

  void onRtkFix(
      const cleanbot_interfaces::msg::RtkFix::SharedPtr message) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    RtkSample sample;
    sample.fixed_valid = message->fixed_valid;
    sample.center_valid = message->center_valid;
    sample.lat = message->lat;
    sample.lon = message->lon;
    sample.heading_deg = message->heading_deg;
    sample.heading_valid = message->heading_valid;
    sample.gga_age_sec = message->gga_age_sec;
    sample.fix_quality = message->fix_quality;
    sample.vehicle_static =
        hardware_received_ &&
        latest_hardware_.connected &&
        latest_hardware_.x_speed == 0 &&
        latest_hardware_.z_speed == 0 &&
        latest_hardware_.brush_speed == 0;
    sample_buffer_.push_back({++rtk_sequence_, sample});
    while (sample_buffer_.size() > 500u) {
      sample_buffer_.pop_front();
    }
    sample_condition_.notify_all();
  }

  static ModelGroup* findGroup(
      CleaningModel& model,
      const std::string& group_id) {
    const auto found = std::find_if(
        model.groups.begin(), model.groups.end(),
        [&group_id](const ModelGroup& group) {
          return group.id == group_id;
        });
    return found == model.groups.end() ? nullptr : &(*found);
  }

  static void updateConfirmation(CleaningModel& model) {
    model.recognition_confirmed =
        !model.groups.empty() &&
        std::all_of(
            model.groups.begin(),
            model.groups.end(),
            [](const ModelGroup& group) {
              return !group.sub_areas.empty() &&
                  std::all_of(
                      group.sub_areas.begin(),
                      group.sub_areas.end(),
                      [](const ModelSubArea& area) {
                        return area.confirmed;
                      }) &&
                  std::all_of(
                      group.connectors.begin(),
                      group.connectors.end(),
                      [](const ModelConnector& connector) {
                        return connector.confirmed;
                      });
            });
  }

  static void invalidateRecognition(ModelGroup& group) {
    group.sub_areas.clear();
    group.connectors.clear();
    group.recognition_status = "unrecognized";
    group.recognition_message.clear();
    group.recognition_confidence = 0.0;
  }

  void failManage(
      const std::shared_ptr<ManageCleaningModel::Response>& response,
      const std::string& code,
      const std::string& message) const {
    response->success = false;
    response->code = code;
    response->message = message;
  }

  void onManage(
      const std::shared_ptr<ManageCleaningModel::Request> request,
      std::shared_ptr<ManageCleaningModel::Response> response) {
    if (!configured_.load()) {
      failManage(response, "CONFIG_NOT_READY", "modeling configuration is not ready");
      return;
    }
    std::lock_guard<std::mutex> lock(repository_mutex_);
    if (!repository_) {
      failManage(response, "MODELING_DATABASE_NOT_READY", "modeling database is not ready");
      return;
    }

    CleaningModel model;
    switch (request->operation) {
      case ManageCleaningModel::Request::OP_CREATE: {
        model.id = request->model_id.empty()
            ? makeId("model")
            : request->model_id;
        model.name = request->name.empty() ? model.id : request->name;
        model.status = "draft";
        const auto result = repository_->save_draft(model);
        if (!result.success) {
          failManage(response, result.code, result.message);
          return;
        }
        break;
      }
      case ManageCleaningModel::Request::OP_LOAD:
        if (!repository_->load_draft(request->model_id, model)) {
          failManage(response, "MODEL_NOT_FOUND", "model draft was not found");
          return;
        }
        break;
      case ManageCleaningModel::Request::OP_LIST:
        response->success = true;
        response->code = "OK";
        response->message = "model list loaded";
        response->model_ids = repository_->list_model_ids();
        return;
      case ManageCleaningModel::Request::OP_DELETE:
        if (!repository_->delete_model(request->model_id)) {
          failManage(response, "MODEL_DELETE_FAILED", "model could not be deleted");
          return;
        }
        response->success = true;
        response->code = "OK";
        response->message = "model deleted";
        return;
      case ManageCleaningModel::Request::OP_CREATE_GROUP: {
        if (!repository_->load_draft(request->model_id, model)) {
          failManage(response, "MODEL_NOT_FOUND", "model draft was not found");
          return;
        }
        ModelGroup group;
        group.id = request->group_id.empty()
            ? makeId("group")
            : request->group_id;
        group.name = request->name.empty() ? group.id : request->name;
        group.area_number =
            static_cast<std::uint32_t>(model.groups.size() + 1u);
        group.sweep_mode =
            request->sweep_mode == "manual" ? "manual" : "auto";
        group.sweep_angle_deg =
            normalize_heading_deg(request->sweep_angle_deg);
        invalidateRecognition(group);
        model.groups.push_back(group);
        model.recognition_confirmed = false;
        model.preview_confirmed = false;
        const auto result = repository_->save_draft(model);
        if (!result.success) {
          failManage(response, result.code, result.message);
          return;
        }
        break;
      }
      case ManageCleaningModel::Request::OP_DELETE_GROUP: {
        if (!repository_->load_draft(request->model_id, model)) {
          failManage(response, "MODEL_NOT_FOUND", "model draft was not found");
          return;
        }
        const auto original_size = model.groups.size();
        model.groups.erase(
            std::remove_if(
                model.groups.begin(),
                model.groups.end(),
                [&request](const ModelGroup& group) {
                  return group.id == request->group_id;
                }),
            model.groups.end());
        if (model.groups.size() == original_size) {
          failManage(response, "GROUP_NOT_FOUND", "model group was not found");
          return;
        }
        updateConfirmation(model);
        model.preview_confirmed = false;
        const auto result = repository_->save_draft(model);
        if (!result.success) {
          failManage(response, result.code, result.message);
          return;
        }
        break;
      }
      case ManageCleaningModel::Request::OP_DELETE_POINT: {
        if (!repository_->load_draft(request->model_id, model)) {
          failManage(response, "MODEL_NOT_FOUND", "model draft was not found");
          return;
        }
        auto* group = findGroup(model, request->group_id);
        if (group == nullptr) {
          failManage(response, "GROUP_NOT_FOUND", "model group was not found");
          return;
        }
        const auto original_size = group->points.size();
        group->points.erase(
            std::remove_if(
                group->points.begin(),
                group->points.end(),
                [&request](const ModelPoint& point) {
                  return point.id == request->point_id;
                }),
            group->points.end());
        if (group->points.size() == original_size) {
          failManage(response, "POINT_NOT_FOUND", "model point was not found");
          return;
        }
        for (std::size_t index = 0u; index < group->points.size(); ++index) {
          group->points[index].sequence = index + 1u;
        }
        invalidateRecognition(*group);
        model.recognition_confirmed = false;
        model.preview_confirmed = false;
        const auto result = repository_->save_draft(model);
        if (!result.success) {
          failManage(response, result.code, result.message);
          return;
        }
        break;
      }
      case ManageCleaningModel::Request::OP_RECOGNIZE_GROUP: {
        if (!repository_->load_draft(request->model_id, model)) {
          failManage(response, "MODEL_NOT_FOUND", "model draft was not found");
          return;
        }
        auto* group = findGroup(model, request->group_id);
        if (group == nullptr) {
          failManage(response, "GROUP_NOT_FOUND", "model group was not found");
          return;
        }
        const auto recognition = recognize_group(*group, recognition_options_);
        if (!recognition.success) {
          *group = recognition.group;
          repository_->save_draft(model);
          failManage(response, recognition.code, recognition.message);
          return;
        }
        *group = recognition.group;
        updateConfirmation(model);
        model.preview_confirmed = false;
        const auto result = repository_->save_draft(model);
        if (!result.success) {
          failManage(response, result.code, result.message);
          return;
        }
        break;
      }
      case ManageCleaningModel::Request::OP_CONFIRM_RECOGNITION: {
        if (!repository_->load_draft(request->model_id, model)) {
          failManage(response, "MODEL_NOT_FOUND", "model draft was not found");
          return;
        }
        auto* group = findGroup(model, request->group_id);
        if (group == nullptr || group->sub_areas.empty()) {
          failManage(response, "RECOGNITION_MISSING", "recognized sub-areas are missing");
          return;
        }
        for (auto& area : group->sub_areas) {
          area.confirmed = true;
        }
        for (auto& connector : group->connectors) {
          connector.confirmed = true;
        }
        group->recognition_status = "recognized";
        group->recognition_message = "recognition manually confirmed";
        updateConfirmation(model);
        model.preview_confirmed = false;
        const auto result = repository_->save_draft(model);
        if (!result.success) {
          failManage(response, result.code, result.message);
          return;
        }
        break;
      }
      case ManageCleaningModel::Request::OP_SAVE_VERSION: {
        if (!repository_->load_draft(request->model_id, model)) {
          failManage(response, "MODEL_NOT_FOUND", "model draft was not found");
          return;
        }
        if (!model.recognition_confirmed) {
          failManage(response, "MODEL_NOT_CONFIRMED", "model recognition is not confirmed");
          return;
        }
        const auto result = repository_->save_formal_version(model);
        if (!result.success ||
            !repository_->load_version(model.id, result.version, model)) {
          failManage(
              response,
              result.success ? "MODEL_VERSION_LOAD_FAILED" : result.code,
              result.success ? "saved model version could not be loaded" : result.message);
          return;
        }
        break;
      }
      default:
        failManage(response, "MODEL_OPERATION_UNKNOWN", "unknown model operation");
        return;
    }

    response->success = true;
    response->code = "OK";
    response->message = "model operation completed";
    response->model = toRosModel(model);
  }

  void onSamplePoint(
      const std::shared_ptr<SampleModelPoint::Request> request,
      std::shared_ptr<SampleModelPoint::Response> response) {
    if (!configured_.load()) {
      response->code = "CONFIG_NOT_READY";
      response->message = "modeling configuration is not ready";
      return;
    }
    const std::string capture_type = request->capture_type.empty()
        ? "boundary"
        : request->capture_type;
    if (capture_type != "boundary" && capture_type != "connection") {
      response->code = "CAPTURE_TYPE_INVALID";
      response->message = "capture type must be boundary or connection";
      return;
    }

    std::vector<RtkSample> samples;
    {
      std::unique_lock<std::mutex> lock(state_mutex_);
      const std::uint64_t first_sequence = rtk_sequence_;
      const auto deadline =
          std::chrono::steady_clock::now() + std::chrono::seconds(3);
      sample_condition_.wait_until(lock, deadline, [this, first_sequence]() {
        const auto count = static_cast<std::size_t>(std::count_if(
            sample_buffer_.begin(),
            sample_buffer_.end(),
            [first_sequence](const BufferedSample& sample) {
              return sample.sequence > first_sequence;
            }));
        return count >= sample_count_;
      });
      for (const auto& buffered : sample_buffer_) {
        if (buffered.sequence > first_sequence) {
          samples.push_back(buffered.sample);
        }
      }
      if (samples.size() > sample_count_) {
        samples.erase(
            samples.begin(),
            samples.end() - static_cast<std::ptrdiff_t>(sample_count_));
      }
    }

    auto sampled =
        sample_point(samples, sample_count_, maximum_sample_radius_m_);
    if (!sampled.success) {
      response->code = sampled.code;
      response->message = sampled.message;
      return;
    }

    std::lock_guard<std::mutex> repository_lock(repository_mutex_);
    CleaningModel model;
    if (!repository_ ||
        !repository_->load_draft(request->model_id, model)) {
      response->code = "MODEL_NOT_FOUND";
      response->message = "model draft was not found";
      return;
    }
    auto* group = findGroup(model, request->group_id);
    if (group == nullptr) {
      response->code = "GROUP_NOT_FOUND";
      response->message = "model group was not found";
      return;
    }

    sampled.point.id = makeId("point");
    sampled.point.sequence = group->points.size() + 1u;
    if (!set_model_point_local_coordinates(model, sampled.point)) {
      response->code = "POINT_COORDINATE_INVALID";
      response->message = "sampled RTK coordinate is invalid";
      return;
    }
    sampled.point.capture_type = capture_type;
    group->points.push_back(sampled.point);
    invalidateRecognition(*group);
    model.recognition_confirmed = false;
    model.preview_confirmed = false;
    const auto write = repository_->save_draft(model);
    if (!write.success) {
      response->code = write.code;
      response->message = write.message;
      return;
    }

    response->success = true;
    response->code = "OK";
    response->message = "model point sampled";
    response->point = toRosPoint(sampled.point);
  }

  void onGeneratePlan(
      const std::shared_ptr<GenerateCleaningPlan::Request> request,
      std::shared_ptr<GenerateCleaningPlan::Response> response) {
    if (!configured_.load()) {
      response->code = "CONFIG_NOT_READY";
      response->message = "modeling configuration is not ready";
      return;
    }
    std::lock_guard<std::mutex> lock(repository_mutex_);
    if (!repository_) {
      response->code = "MODELING_DATABASE_NOT_READY";
      response->message = "modeling database is not ready";
      return;
    }

    CleaningModel model;
    std::uint64_t version = request->model_version;
    if (version == 0u) {
      if (!repository_->load_draft(request->model_id, model)) {
        response->code = "MODEL_NOT_FOUND";
        response->message = "model draft was not found";
        return;
      }
      const auto saved = repository_->save_formal_version(model);
      if (!saved.success) {
        response->code = saved.code;
        response->message = saved.message;
        return;
      }
      version = saved.version;
    }
    if (!repository_->load_version(request->model_id, version, model)) {
      response->code = "MODEL_VERSION_NOT_FOUND";
      response->message = "formal model version was not found";
      return;
    }

    const auto built = build_task_plan(
        model,
        brush_width_cm_,
        minimum_overlap_cm_,
        request->speed > 0 ? request->speed : base_forward_speed_);
    if (!built.success) {
      response->code = built.code;
      response->message = built.message;
      return;
    }
    auto plan = built.plan;
    plan.generated_at =
        static_cast<std::uint64_t>(now().seconds());
    plan.preview_confirmed = request->confirm_preview;
    const auto write = repository_->save_plan(plan);
    if (!write.success) {
      response->code = write.code;
      response->message = write.message;
      return;
    }

    response->success = true;
    response->code = "OK";
    response->message = request->confirm_preview
        ? "cleaning plan generated and confirmed"
        : "cleaning plan generated; preview confirmation required";
    response->plan = toRosPlan(plan);
  }

  void onExecutePlan(
      const std::shared_ptr<ExecuteModelPlan::Request> request,
      std::shared_ptr<ExecuteModelPlan::Response> response) {
    if (!configured_.load()) {
      response->code = "CONFIG_NOT_READY";
      response->message = "modeling configuration is not ready";
      return;
    }
    if (request->brush_speed <= 0) {
      response->code = "BRUSH_SPEED_INVALID";
      response->message = "brush speed must be positive";
      return;
    }

    CleaningPlan plan;
    {
      std::lock_guard<std::mutex> lock(repository_mutex_);
      if (!repository_ || !repository_->load_plan(request->plan_id, plan)) {
        response->code = "PLAN_NOT_FOUND";
        response->message = "cleaning plan was not found";
        return;
      }
    }
    if (!plan.preview_confirmed) {
      response->code = "PLAN_PREVIEW_NOT_CONFIRMED";
      response->message = "plan preview must be confirmed before execution";
      return;
    }
    if (plan.segments.empty()) {
      response->code = "PLAN_EMPTY";
      response->message = "cleaning plan has no segments";
      return;
    }
    if (!execute_client_->wait_for_action_server(
            std::chrono::seconds(0))) {
      response->code = "MISSION_ACTION_UNAVAILABLE";
      response->message = "cleaning mission action server is unavailable";
      return;
    }

    ExecuteCleaning::Goal goal;
    goal.task_id = plan.id;
    goal.model_id = plan.model_id;
    goal.model_version = plan.model_version;
    goal.plan_id = plan.id;
    goal.plan_hash = plan.plan_hash;
    goal.brush_speed = request->brush_speed;
    goal.loop = false;
    goal.loop_count = 0u;
    for (const auto& segment : plan.segments) {
      goal.segments.push_back(toRosSegment(segment));
    }

    const auto apply_submission =
        [&response](const GoalSubmissionDecision& decision) {
          response->accepted = decision.accepted;
          response->code = decision.code;
          response->message = decision.message;
        };
    auto response_timed_out = std::make_shared<std::atomic<bool>>(false);
    auto late_cancel_requested =
        std::make_shared<std::atomic<bool>>(false);
    rclcpp_action::Client<ExecuteCleaning>::SendGoalOptions options;
    options.goal_response_callback =
        [this, response_timed_out, late_cancel_requested](
            const auto& goal_handle) {
          if (!goal_handle) {
            RCLCPP_ERROR(
                get_logger(),
                "generated cleaning plan was rejected by mission manager");
            return;
          }
          if (response_timed_out->load()) {
            if (!late_cancel_requested->exchange(true)) {
              RCLCPP_WARN(
                  get_logger(),
                  "mission goal was accepted after execute-plan response timed out; "
                  "requesting cancellation");
              execute_client_->async_cancel_goal(goal_handle);
            }
          }
        };
    options.result_callback =
        [this](const auto& result) {
          if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_WARN(
                get_logger(),
                "generated cleaning plan finished with action result %d",
                static_cast<int>(result.code));
          }
        };
    const auto goal_future = execute_client_->async_send_goal(goal, options);
    if (goal_future.wait_for(kGoalResponseDeadline) !=
        std::future_status::ready) {
      response_timed_out->store(true);
      // Close the race where the response became ready immediately after the
      // first wait returned but before the timeout marker was observed.
      if (goal_future.wait_for(std::chrono::seconds(0)) ==
          std::future_status::ready) {
        const auto late_goal = goal_future.get();
        if (late_goal && !late_cancel_requested->exchange(true)) {
          execute_client_->async_cancel_goal(late_goal);
        }
      }
      apply_submission(
          classify_goal_response(GoalResponseState::kTimeout));
      return;
    }

    const auto goal_handle = goal_future.get();
    if (!goal_handle) {
      apply_submission(
          classify_goal_response(GoalResponseState::kRejected));
      return;
    }
    apply_submission(
        classify_goal_response(GoalResponseState::kAccepted));
  }

  std::unique_ptr<config::ConfigClient> config_client_;
  std::unique_ptr<SqliteModelRepository> repository_;
  std::atomic<bool> configured_{false};
  std::atomic<std::uint64_t> id_sequence_{0u};

  std::mutex repository_mutex_;
  std::mutex state_mutex_;
  std::condition_variable sample_condition_;
  cleanbot_interfaces::msg::HardwareStatus latest_hardware_;
  bool hardware_received_{false};
  std::uint64_t rtk_sequence_{0u};
  std::deque<BufferedSample> sample_buffer_;

  double brush_width_cm_{116.0};
  double minimum_overlap_cm_{10.0};
  std::size_t sample_count_{10u};
  double maximum_sample_radius_m_{0.05};
  RecognitionOptions recognition_options_;
  std::int32_t base_forward_speed_{350};

  rclcpp::CallbackGroup::SharedPtr io_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr action_group_;
  rclcpp::Subscription<cleanbot_interfaces::msg::RtkFix>::SharedPtr
      rtk_subscription_;
  rclcpp::Subscription<cleanbot_interfaces::msg::HardwareStatus>::SharedPtr
      hardware_subscription_;
  rclcpp::Service<ManageCleaningModel>::SharedPtr manage_service_;
  rclcpp::Service<SampleModelPoint>::SharedPtr sample_service_;
  rclcpp::Service<GenerateCleaningPlan>::SharedPtr generate_service_;
  rclcpp::Service<ExecuteModelPlan>::SharedPtr execute_service_;
  rclcpp_action::Client<ExecuteCleaning>::SharedPtr execute_client_;
};

}  // namespace modeling
}  // namespace cleanbot

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cleanbot::modeling::ModelingManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 3u);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
