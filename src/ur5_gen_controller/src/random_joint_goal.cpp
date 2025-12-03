#include <memory>
#include <thread>
#include <vector>
#include <random>
#include <optional>
#include <string>
#include <regex>
#include <curl/curl.h>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <ur5_teleop_msgs/msg/pose_delta.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
struct PoseDelta
{
  double dx;
  double dy;
  double dz;
  double droll;
  double dpitch;
  double dyaw;
};

struct CurlGlobalInit
{
  CurlGlobalInit() : result(curl_global_init(CURL_GLOBAL_DEFAULT)) {}
  ~CurlGlobalInit()
  {
    if (result == CURLE_OK)
    {
      curl_global_cleanup();
    }
  }
  bool ok() const { return result == CURLE_OK; }

private:
  CURLcode result;
};

size_t curlWriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
  const size_t total = size * nmemb;
  auto *buffer = static_cast<std::string *>(userp);
  buffer->append(static_cast<char *>(contents), total);
  return total;
}

bool parsePoseDelta(const std::string &json, PoseDelta &delta)
{
  const auto joints_pos = json.find("\"joints\"");
  if (joints_pos == std::string::npos)
  {
    return false;
  }

  const auto start = json.find('{', joints_pos);
  if (start == std::string::npos)
  {
    return false;
  }

  size_t end = start;
  int depth = 1;
  while (depth > 0 && ++end < json.size())
  {
    if (json[end] == '{')
    {
      ++depth;
    }
    else if (json[end] == '}')
    {
      --depth;
    }
  }

  if (depth != 0)
  {
    return false;
  }

  const std::string joints_block = json.substr(start, end - start + 1);

  auto extract = [&](const std::string &key, double &value) {
    std::regex pattern("\"" + key + "\"\\s*:\\s*(-?\\d+(?:\\.\\d+)?(?:[eE][+-]?\\d+)?)");
    std::smatch match;
    if (std::regex_search(joints_block, match, pattern))
    {
      value = std::stod(match[1].str());
      return true;
    }
    return false;
  };

  return extract("x", delta.dx) &&
         extract("y", delta.dy) &&
         extract("z", delta.dz) &&
         extract("roll", delta.droll) &&
         extract("pitch", delta.dpitch) &&
         extract("yaw", delta.dyaw);
}

std::optional<PoseDelta> fetchPoseDelta(const std::string &url, const rclcpp::Logger &logger)
{
  CURL *curl = curl_easy_init();
  if (!curl)
  {
    RCLCPP_WARN(logger, "Unable to create CURL easy handle.");
    return std::nullopt;
  }

  std::string response;
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 1L);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlWriteCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

  CURLcode res = curl_easy_perform(curl);
  long status = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK || status != 200 || response.empty())
  {
    RCLCPP_DEBUG(logger, "Action server response unavailable (HTTP %ld, curl %d).", status, static_cast<int>(res));
    return std::nullopt;
  }

  PoseDelta delta{};
  if (!parsePoseDelta(response, delta))
  {
    RCLCPP_WARN(logger, "Failed to parse action payload: %s", response.c_str());
    return std::nullopt;
  }

  return delta;
}
}  // namespace

class UR5TeleopController : public rclcpp::Node
{
public:
  UR5TeleopController() : Node("ur5_teleop_controller")
  {
    // Subscribe to teleop delta topic
    subscription_ = this->create_subscription<ur5_teleop_msgs::msg::PoseDelta>(
      "/ur5/teleop_delta", 10,
      std::bind(&UR5TeleopController::teleopCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "UR5 Teleop Controller: Subscribed to /ur5/teleop_delta");
  }

  std::optional<PoseDelta> getLatestDelta()
  {
    std::lock_guard<std::mutex> lock(delta_mutex_);
    if (has_new_delta_)
    {
      has_new_delta_ = false;
      return latest_delta_;
    }
    return std::nullopt;
  }

private:
  void teleopCallback(const ur5_teleop_msgs::msg::PoseDelta::SharedPtr msg)
  {
    // Log received message for debugging
    RCLCPP_INFO(this->get_logger(), 
                "Received delta: dx=%.6f, dy=%.6f, dz=%.6f, dr=%.6f, dp=%.6f, dyw=%.6f",
                msg->dx, msg->dy, msg->dz, msg->droll, msg->dpitch, msg->dyaw);
    
    // Ignore zero deltas (stop commands)
    if (std::abs(msg->dx) < 1e-6 && std::abs(msg->dy) < 1e-6 && std::abs(msg->dz) < 1e-6 &&
        std::abs(msg->droll) < 1e-6 && std::abs(msg->dpitch) < 1e-6 && std::abs(msg->dyaw) < 1e-6)
    {
      RCLCPP_DEBUG(this->get_logger(), "Ignoring zero delta");
      return;
    }
    
    std::lock_guard<std::mutex> lock(delta_mutex_);
    latest_delta_.dx = msg->dx;
    latest_delta_.dy = msg->dy;
    latest_delta_.dz = msg->dz;
    latest_delta_.droll = msg->droll;
    latest_delta_.dpitch = msg->dpitch;
    latest_delta_.dyaw = msg->dyaw;
    has_new_delta_ = true;
    RCLCPP_INFO(this->get_logger(), "Delta stored for execution");
  }

  rclcpp::Subscription<ur5_teleop_msgs::msg::PoseDelta>::SharedPtr subscription_;
  std::mutex delta_mutex_;
  PoseDelta latest_delta_;
  bool has_new_delta_ = false;
};

int main(int argc, char **argv)
{
  // Initialize ROS 2 runtime and executor context
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<UR5TeleopController>();

  CurlGlobalInit curl_init;
  if (!curl_init.ok())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize libcurl.");
    return 1;
  }

  // Prepare single-threaded executor for the node
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // Configure MoveIt! interface for the UR manipulator
  static const std::string planning_group = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
  move_group.setEndEffectorLink("tool0");  // Explicitly set the end-effector
  
  // Declare parameters
  const std::string action_server_url =
    node->declare_parameter<std::string>("action_server_url", "http://localhost:5000/joint_actions");
  const bool use_http_server = 
    node->declare_parameter<bool>("use_http_server", false);
  const bool use_random_motion =
    node->declare_parameter<bool>("use_random_motion", false);
  const int max_iterations =
    node->declare_parameter<int>("max_iterations", 1000);
  const std::string control_frame =
    node->declare_parameter<std::string>("control_frame", "tool"); // "tool" or "base"

  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);
  move_group.setPlanningTime(1.0);  // Reduce planning time for faster, more local solutions
  move_group.setNumPlanningAttempts(2); // Fewer attempts are needed for small jogs
  move_group.setGoalJointTolerance(1e-3);

  // Retrieve current robot state and ensure availability
  if (!move_group.getCurrentState(5.0))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to retrieve the current robot state");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Configuration:");
  RCLCPP_INFO(node->get_logger(), "  - Use HTTP Server: %s", use_http_server ? "YES" : "NO");
  RCLCPP_INFO(node->get_logger(), "  - Use Random Motion: %s", use_random_motion ? "YES" : "NO");
    RCLCPP_INFO(node->get_logger(), "  - Use Topic Subscription: YES (always active)");
  RCLCPP_INFO(node->get_logger(), "  - Max Iterations: %d", max_iterations);
  RCLCPP_INFO(node->get_logger(), "  - Control Frame: %s", control_frame.c_str());

  rclcpp::Rate rate(10.0);  // 10Hz loop rate for responsive teleoperation
  
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> position_delta(-0.02, 0.02);
  std::uniform_real_distribution<double> orientation_delta(-0.05, 0.05);

  bool all_success = true;
  int iteration = 0;

  while (rclcpp::ok() && iteration < max_iterations)
  {
    // Set the start state for planning to the current state
    move_group.setStartStateToCurrentState();

    auto current_pose_stamped = move_group.getCurrentPose();
    auto target_pose = current_pose_stamped.pose;

    double dx = 0.0;
    double dy = 0.0;
    double dz = 0.0;
    double droll = 0.0;
    double dpitch = 0.0;
    double dyaw = 0.0;
    bool has_delta = false;
    std::string source = "none";

    // Priority 1: Check for topic message (teleop)
    auto topic_delta = node->getLatestDelta();
    if (topic_delta.has_value())
    {
      dx = topic_delta->dx;
      dy = topic_delta->dy;
      dz = topic_delta->dz;
      droll = topic_delta->droll;
      dpitch = topic_delta->dpitch;
      dyaw = topic_delta->dyaw;
      has_delta = true;
      source = "topic";
      iteration++;
    }
    // Priority 2: Check HTTP server if enabled
    else if (use_http_server)
    {
      auto server_delta = fetchPoseDelta(action_server_url, node->get_logger());
      if (server_delta.has_value())
      {
        dx = server_delta->dx;
        dy = server_delta->dy;
        dz = server_delta->dz;
        droll = server_delta->droll;
        dpitch = server_delta->dpitch;
        dyaw = server_delta->dyaw;
        has_delta = true;
        source = "http";
        iteration++;
      }
    }
    // Priority 3: Use random motion if enabled
    else if (use_random_motion)
    {
      dx = position_delta(gen);
      dy = position_delta(gen);
      dz = position_delta(gen);
      droll = orientation_delta(gen);
      dpitch = orientation_delta(gen);
      dyaw = orientation_delta(gen);
      has_delta = true;
      source = "random";
      iteration++;
    }

    // Only execute motion if we have a delta
    if (!has_delta)
    {
      rate.sleep();
      continue;
    }

    RCLCPP_DEBUG(node->get_logger(), "Current Pose: pos(%.3f, %.3f, %.3f), quat(%.3f, %.3f, %.3f, %.3f)",
                target_pose.position.x, target_pose.position.y, target_pose.position.z,
                target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z,
                target_pose.orientation.w);

    tf2::Quaternion current_quat;
    tf2::fromMsg(current_pose_stamped.pose.orientation, current_quat);

    tf2::Quaternion delta_quat;
    delta_quat.setRPY(droll, dpitch, dyaw);
    tf2::Quaternion target_quat;

    if (control_frame == "base")
    {
      // Apply translation in base frame (global axes)
      target_pose.position.x += dx;
      target_pose.position.y += dy;
      target_pose.position.z += dz;

      // Apply rotation in base frame (pre-multiply)
      target_quat = delta_quat * current_quat;
    }
    else // "tool" or default
    {
      // Transform translational deltas from local (end-effector) frame to planning frame
      tf2::Vector3 local_delta(dx, dy, dz);
      tf2::Vector3 global_delta = tf2::quatRotate(current_quat, local_delta);

      target_pose.position.x += global_delta.x();
      target_pose.position.y += global_delta.y();
      target_pose.position.z += global_delta.z();

      // Apply rotation in local frame (post-multiply)
      target_quat = current_quat * delta_quat;
    }

    target_quat.normalize();
    target_pose.orientation = tf2::toMsg(target_quat);

    RCLCPP_DEBUG(node->get_logger(),
                "Iteration %d (%s) [%s frame]: Δpos = (%.3f, %.3f, %.3f) m, Δrpy = (%.3f, %.3f, %.3f) rad",
                iteration, source.c_str(), control_frame.c_str(),
                dx, dy, dz, droll, dpitch, dyaw);

    RCLCPP_DEBUG(node->get_logger(), "Target Pose:  pos(%.3f, %.3f, %.3f), quat(%.3f, %.3f, %.3f, %.3f)",
                target_pose.position.x, target_pose.position.y, target_pose.position.z,
                target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z,
                target_pose.orientation.w);

    // Use Cartesian path planning for smooth, direct motion
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;  // 1cm resolution for Cartesian path
    const double jump_threshold = 0.0;  // Disable jump threshold for small motions
    
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction >= 0.99)  // At least 99% of path successfully planned
    {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      
      // Execute synchronously to avoid race conditions
      auto result = move_group.execute(plan);
      if (result != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_DEBUG(node->get_logger(), "Execution failed on iteration %d", iteration);
      }
    }
    else
    {
      all_success = false;
      RCLCPP_DEBUG(node->get_logger(),
                  "Cartesian planning achieved only %.2f%% of the path on iteration %d. Skipping this delta.",
                  fraction * 100.0, iteration);
    }
    rate.sleep();
  }

  if (all_success)
  {
    RCLCPP_INFO(node->get_logger(), "Completed end-effector jogging sequence.");
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "Jogging sequence finished with one or more failures.");
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return all_success ? 0 : 2;
}
