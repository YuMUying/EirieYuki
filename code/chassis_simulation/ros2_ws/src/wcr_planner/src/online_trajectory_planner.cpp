#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "wcr_planning_msgs/msg/obstacle.hpp"
#include "wcr_planning_msgs/msg/obstacle_array.hpp"
#include "wcr_planning_msgs/msg/obstacle_update.hpp"
#include "wcr_planning_msgs/msg/timed_trajectory.hpp"
#include "wcr_planning_msgs/msg/trajectory_point.hpp"

namespace
{
constexpr std::uint8_t kGoalMode = 1;
constexpr std::uint8_t kCurveMode = 2;
constexpr double kEpsilon = 1e-9;

struct Point2
{
  double x{0.0};
  double y{0.0};
};

double Distance(const Point2 &a, const Point2 &b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double YawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm < kEpsilon) {
    return 0.0;
  }
  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

geometry_msgs::msg::Quaternion QuaternionFromYaw(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}
}  // namespace

class OnlineTrajectoryPlanner final : public rclcpp::Node
{
public:
  OnlineTrajectoryPlanner()
  : Node("online_trajectory_planner")
  {
    this->declare_parameter("mode_topic", "/wcr/planning_mode");
    this->declare_parameter("obstacles_topic", "/wcr/obstacles");
    this->declare_parameter("obstacle_update_topic", "/wcr/obstacle_update");
    this->declare_parameter("goal_topic", "/wcr/target_pose");
    this->declare_parameter("reference_curve_topic", "/wcr/reference_curve");
    this->declare_parameter("odom_topic", "/wcr/odom");
    this->declare_parameter("controller_target_topic", "/wcr/controller_target_pose");
    this->declare_parameter("controller_cancel_topic", "/wcr/controller_cancel");
    this->declare_parameter("controller_reached_topic", "/wcr/target_reached");
    this->declare_parameter("planned_path_topic", "/wcr/planned_path");
    this->declare_parameter("trajectory_topic", "/wcr/planned_trajectory");
    this->declare_parameter("status_topic", "/wcr/planner_status");
    this->declare_parameter("reached_output_topic", "/wcr/target_reached");
    this->declare_parameter("default_mode", static_cast<int>(kGoalMode));
    this->declare_parameter("grid_resolution", 0.03);
    this->declare_parameter("planning_padding", 0.75);
    this->declare_parameter("robot_radius", 0.18);
    this->declare_parameter("safety_margin", 0.02);
    this->declare_parameter("tracking_error_allowance", 0.05);
    this->declare_parameter("collision_sample_step", 0.01);
    this->declare_parameter("waypoint_tolerance", 0.025);
    this->declare_parameter("trajectory_sample_distance", 0.015);
    this->declare_parameter("curve_smoothing_iterations", 3);
    this->declare_parameter("curve_smoothing_weight", 0.20);
    this->declare_parameter("max_trajectory_speed", 0.13);
    this->declare_parameter("max_lateral_acceleration", 0.18);
    this->declare_parameter("max_acceleration", 0.12);
    this->declare_parameter("max_deceleration", 0.16);
    this->declare_parameter("minimum_cruise_speed", 0.035);
    this->declare_parameter("max_grid_cells", 250000);
    this->declare_parameter("max_expansions", 200000);

    mode_ = static_cast<std::uint8_t>(this->get_parameter("default_mode").as_int());
    if (mode_ != kGoalMode && mode_ != kCurveMode) {
      mode_ = kGoalMode;
    }

    auto inputQos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto stateQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    modeSubscription_ = this->create_subscription<std_msgs::msg::UInt8>(
      this->get_parameter("mode_topic").as_string(), inputQos,
      [this](const std_msgs::msg::UInt8::SharedPtr message)
      {
        if (message->data != kGoalMode && message->data != kCurveMode) {
          PublishStatus("rejected_mode: expected 1 (goal) or 2 (curve)");
          return;
        }
        mode_ = message->data;
        PublishStatus(mode_ == kGoalMode ? "mode_1_goal" : "mode_2_curve");
        RequestReplan("mode changed");
      });

    obstacleArraySubscription_ =
      this->create_subscription<wcr_planning_msgs::msg::ObstacleArray>(
      this->get_parameter("obstacles_topic").as_string(), inputQos,
      [this](const wcr_planning_msgs::msg::ObstacleArray::SharedPtr message)
      {
        obstacles_.clear();
        std::size_t generatedId = 0;
        for (const auto &obstacle : message->obstacles) {
          if (!ValidObstacle(obstacle)) {
            RCLCPP_WARN(this->get_logger(), "Ignoring invalid obstacle [%s].", obstacle.id.c_str());
            continue;
          }
          auto stored = obstacle;
          if (stored.id.empty()) {
            stored.id = "snapshot_" + std::to_string(generatedId++);
          }
          obstacles_[stored.id] = std::move(stored);
        }
        RequestReplan("obstacle snapshot updated");
      });

    obstacleUpdateSubscription_ =
      this->create_subscription<wcr_planning_msgs::msg::ObstacleUpdate>(
      this->get_parameter("obstacle_update_topic").as_string(), inputQos,
      [this](const wcr_planning_msgs::msg::ObstacleUpdate::SharedPtr message)
      {
        using Update = wcr_planning_msgs::msg::ObstacleUpdate;
        if (message->operation == Update::CLEAR) {
          obstacles_.clear();
        } else if (message->operation == Update::REMOVE) {
          if (message->obstacle.id.empty()) {
            PublishStatus("rejected_obstacle_remove: id is empty");
            return;
          }
          obstacles_.erase(message->obstacle.id);
        } else if (message->operation == Update::UPSERT) {
          if (message->obstacle.id.empty() || !ValidObstacle(message->obstacle)) {
            PublishStatus("rejected_obstacle_upsert: invalid id, shape, or size");
            return;
          }
          obstacles_[message->obstacle.id] = message->obstacle;
        } else {
          PublishStatus("rejected_obstacle_update: unknown operation");
          return;
        }
        RequestReplan("obstacle increment updated");
      });

    goalSubscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      this->get_parameter("goal_topic").as_string(), inputQos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message)
      {
        if (!FrameAccepted(message->header.frame_id)) {
          return;
        }
        goal_ = *message;
        goal_.header.frame_id = "odom";
        hasGoal_ = true;
        if (mode_ == kGoalMode) {
          RequestReplan("goal updated");
        }
      });

    curveSubscription_ = this->create_subscription<nav_msgs::msg::Path>(
      this->get_parameter("reference_curve_topic").as_string(), inputQos,
      [this](const nav_msgs::msg::Path::SharedPtr message)
      {
        if (!FrameAccepted(message->header.frame_id)) {
          return;
        }
        referenceCurve_ = *message;
        referenceCurve_.header.frame_id = "odom";
        for (auto &pose : referenceCurve_.poses) {
          pose.header.frame_id = "odom";
        }
        hasCurve_ = !referenceCurve_.poses.empty();
        if (mode_ == kCurveMode) {
          RequestReplan("reference curve updated");
        }
      });

    odomSubscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->get_parameter("odom_topic").as_string(), rclcpp::QoS(20),
      [this](const nav_msgs::msg::Odometry::SharedPtr message)
      {
        odometry_ = *message;
        hasOdometry_ = true;
        if (executing_ &&
            (IsBlocked(CurrentPosition(), SafetyInflation()) ||
             (hasPreviousExecutionPosition_ &&
              !SegmentFree(previousExecutionPosition_, CurrentPosition(), SafetyInflation())))) {
          StopForSafetyViolation();
        }
        previousExecutionPosition_ = CurrentPosition();
        hasPreviousExecutionPosition_ = true;
        if (replanPending_) {
          Replan();
        }
      });

    reachedSubscription_ = this->create_subscription<std_msgs::msg::Bool>(
      this->get_parameter("controller_reached_topic").as_string(), rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr message)
      {
        if (!message->data || !executing_ || !hasOdometry_ || activePath_.poses.empty() ||
            currentWaypoint_ + 1 < activePath_.poses.size()) {
          return;
        }
        const Point2 finalPoint = PosePoint(activePath_.poses.back());
        if (Distance(CurrentPosition(), finalPoint) <=
            this->get_parameter("waypoint_tolerance").as_double()) {
          executing_ = false;
          PublishReached(true);
          PublishStatus("completed");
        }
      });

    controllerTargetPublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      this->get_parameter("controller_target_topic").as_string(), rclcpp::QoS(10));
    controllerCancelPublisher_ = this->create_publisher<std_msgs::msg::Bool>(
      this->get_parameter("controller_cancel_topic").as_string(), rclcpp::QoS(10));
    pathPublisher_ = this->create_publisher<nav_msgs::msg::Path>(
      this->get_parameter("planned_path_topic").as_string(), stateQos);
    trajectoryPublisher_ = this->create_publisher<wcr_planning_msgs::msg::TimedTrajectory>(
      this->get_parameter("trajectory_topic").as_string(), stateQos);
    statusPublisher_ = this->create_publisher<std_msgs::msg::String>(
      this->get_parameter("status_topic").as_string(), stateQos);
    reachedPublisher_ = this->create_publisher<std_msgs::msg::Bool>(
      this->get_parameter("reached_output_topic").as_string(), stateQos);

    executionTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&OnlineTrajectoryPlanner::UpdateExecution, this));
    PublishStatus(mode_ == kGoalMode ? "waiting_for_mode_1_goal" : "waiting_for_mode_2_curve");
  }

private:
  bool FrameAccepted(const std::string &frame)
  {
    if (frame.empty() || frame == "odom") {
      return true;
    }
    PublishStatus("rejected_frame: planner accepts only odom coordinates");
    return false;
  }

  static bool ValidObstacle(const wcr_planning_msgs::msg::Obstacle &obstacle)
  {
    using Obstacle = wcr_planning_msgs::msg::Obstacle;
    if (!std::isfinite(obstacle.center.x) || !std::isfinite(obstacle.center.y)) {
      return false;
    }
    if (obstacle.shape == Obstacle::CIRCLE) {
      return std::isfinite(obstacle.radius) && obstacle.radius > 0.0;
    }
    if (obstacle.shape == Obstacle::RECTANGLE) {
      return std::isfinite(obstacle.width) && std::isfinite(obstacle.height) &&
             std::isfinite(obstacle.yaw) && obstacle.width > 0.0 && obstacle.height > 0.0;
    }
    return false;
  }

  Point2 CurrentPosition() const
  {
    return {odometry_.pose.pose.position.x, odometry_.pose.pose.position.y};
  }

  double SafetyInflation() const
  {
    return this->get_parameter("robot_radius").as_double() +
      this->get_parameter("safety_margin").as_double();
  }

  double PlanningInflation() const
  {
    return SafetyInflation() +
      std::max(0.0, this->get_parameter("tracking_error_allowance").as_double());
  }

  bool IsBlocked(const Point2 &point, const double inflation) const
  {
    using Obstacle = wcr_planning_msgs::msg::Obstacle;
    for (const auto &[id, obstacle] : obstacles_) {
      (void)id;
      const double dx = point.x - obstacle.center.x;
      const double dy = point.y - obstacle.center.y;
      if (obstacle.shape == Obstacle::CIRCLE) {
        if (std::hypot(dx, dy) <= obstacle.radius + inflation) {
          return true;
        }
      } else if (obstacle.shape == Obstacle::RECTANGLE) {
        const double cosine = std::cos(obstacle.yaw);
        const double sine = std::sin(obstacle.yaw);
        const double localX = cosine * dx + sine * dy;
        const double localY = -sine * dx + cosine * dy;
        if (std::abs(localX) <= obstacle.width * 0.5 + inflation &&
            std::abs(localY) <= obstacle.height * 0.5 + inflation) {
          return true;
        }
      }
    }
    return false;
  }

  bool SegmentFree(
    const Point2 &start, const Point2 &goal, const double inflation) const
  {
    const double distance = Distance(start, goal);
    const double requestedStep = this->get_parameter("collision_sample_step").as_double();
    const double resolution = this->get_parameter("grid_resolution").as_double();
    const double step = std::max(0.001, std::min(requestedStep, resolution * 0.5));
    const int samples = std::max(1, static_cast<int>(std::ceil(distance / step)));
    for (int i = 0; i <= samples; ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(samples);
      if (IsBlocked(
          {start.x + ratio * (goal.x - start.x),
           start.y + ratio * (goal.y - start.y)}, inflation)) {
        return false;
      }
    }
    return true;
  }

  bool PlanSegment(const Point2 &start, const Point2 &goal, std::vector<Point2> &result)
  {
    result.clear();
    const double planningInflation = PlanningInflation();
    if (IsBlocked(start, planningInflation) || IsBlocked(goal, planningInflation)) {
      return false;
    }
    if (SegmentFree(start, goal, planningInflation)) {
      result = {start, goal};
      return true;
    }

    const double resolution = std::max(0.005, this->get_parameter("grid_resolution").as_double());
    double obstacleExtent = 0.0;
    for (const auto &[id, obstacle] : obstacles_) {
      (void)id;
      if (obstacle.shape == wcr_planning_msgs::msg::Obstacle::CIRCLE) {
        obstacleExtent = std::max(obstacleExtent, obstacle.radius + planningInflation);
      } else {
        obstacleExtent = std::max(
          obstacleExtent,
          std::hypot(obstacle.width * 0.5, obstacle.height * 0.5) + planningInflation);
      }
    }
    const double padding = std::max({
      this->get_parameter("planning_padding").as_double(),
      this->get_parameter("robot_radius").as_double() * 2.0,
      obstacleExtent + 2.0 * resolution});
    const double minX = std::min(start.x, goal.x) - padding;
    const double minY = std::min(start.y, goal.y) - padding;
    const double maxX = std::max(start.x, goal.x) + padding;
    const double maxY = std::max(start.y, goal.y) + padding;
    const int width = static_cast<int>(std::ceil((maxX - minX) / resolution)) + 1;
    const int height = static_cast<int>(std::ceil((maxY - minY) / resolution)) + 1;
    const std::int64_t cellCount = static_cast<std::int64_t>(width) * height;
    if (width <= 0 || height <= 0 ||
        cellCount > this->get_parameter("max_grid_cells").as_int()) {
      return false;
    }

    auto toIndex = [width](const int x, const int y) {return y * width + x;};
    auto toPoint = [minX, minY, resolution](const int x, const int y)
      {return Point2{minX + x * resolution, minY + y * resolution};};
    auto toCell = [minX, minY, resolution, width, height](const Point2 &point)
      {
        const int x = std::clamp(
          static_cast<int>(std::lround((point.x - minX) / resolution)), 0, width - 1);
        const int y = std::clamp(
          static_cast<int>(std::lround((point.y - minY) / resolution)), 0, height - 1);
        return std::pair<int, int>{x, y};
      };

    const auto [startX, startY] = toCell(start);
    const auto [goalX, goalY] = toCell(goal);
    const int startIndex = toIndex(startX, startY);
    const int goalIndex = toIndex(goalX, goalY);
    std::vector<double> cost(static_cast<std::size_t>(cellCount),
      std::numeric_limits<double>::infinity());
    std::vector<int> parent(static_cast<std::size_t>(cellCount), -1);
    std::vector<std::uint8_t> closed(static_cast<std::size_t>(cellCount), 0);
    using QueueEntry = std::pair<double, int>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
    cost[startIndex] = 0.0;
    open.emplace(Distance(start, goal), startIndex);

    constexpr std::array<std::pair<int, int>, 8> neighbors{{
      {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
      {1, 0}, {-1, 1}, {0, 1}, {1, 1}}};
    int expansions = 0;
    const int maxExpansions = this->get_parameter("max_expansions").as_int();
    bool found = false;
    while (!open.empty() && expansions < maxExpansions) {
      const int current = open.top().second;
      open.pop();
      if (closed[current]) {
        continue;
      }
      closed[current] = 1;
      ++expansions;
      if (current == goalIndex) {
        found = true;
        break;
      }
      const int currentX = current % width;
      const int currentY = current / width;
      for (const auto &[dx, dy] : neighbors) {
        const int nextX = currentX + dx;
        const int nextY = currentY + dy;
        if (nextX < 0 || nextX >= width || nextY < 0 || nextY >= height) {
          continue;
        }
        const int next = toIndex(nextX, nextY);
        const Point2 currentPoint = current == startIndex ?
          start : toPoint(currentX, currentY);
        const Point2 nextPoint = next == goalIndex ?
          goal : toPoint(nextX, nextY);
        if (closed[next] || IsBlocked(nextPoint, planningInflation) ||
            !SegmentFree(currentPoint, nextPoint, planningInflation)) {
          continue;
        }
        if (dx != 0 && dy != 0 &&
            (IsBlocked(toPoint(currentX + dx, currentY), planningInflation) ||
             IsBlocked(toPoint(currentX, currentY + dy), planningInflation))) {
          continue;
        }
        const double stepCost = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
        const double candidate = cost[current] + stepCost;
        if (candidate + kEpsilon >= cost[next]) {
          continue;
        }
        cost[next] = candidate;
        parent[next] = current;
        open.emplace(candidate + Distance(nextPoint, goal) / resolution, next);
      }
    }
    if (!found) {
      return false;
    }

    std::vector<Point2> raw;
    for (int index = goalIndex; index != -1; index = parent[index]) {
      raw.push_back(toPoint(index % width, index / width));
      if (index == startIndex) {
        break;
      }
    }
    if (raw.empty()) {
      return false;
    }
    std::reverse(raw.begin(), raw.end());
    raw.front() = start;
    raw.back() = goal;

    result.push_back(raw.front());
    std::size_t anchor = 0;
    while (anchor + 1 < raw.size()) {
      std::size_t furthest = raw.size() - 1;
      while (furthest > anchor + 1 &&
             !SegmentFree(raw[anchor], raw[furthest], planningInflation)) {
        --furthest;
      }
      if (!SegmentFree(raw[anchor], raw[furthest], planningInflation)) {
        result.clear();
        return false;
      }
      result.push_back(raw[furthest]);
      anchor = furthest;
    }
    return true;
  }

  static Point2 PosePoint(const geometry_msgs::msg::PoseStamped &pose)
  {
    return {pose.pose.position.x, pose.pose.position.y};
  }

  bool AppendSegment(
    const Point2 &from, const Point2 &to, std::vector<Point2> &combined,
    std::string &error)
  {
    std::vector<Point2> segment;
    if (!PlanSegment(from, to, segment)) {
      error = "no collision-free path between waypoints";
      return false;
    }
    if (combined.empty()) {
      combined = std::move(segment);
    } else {
      combined.insert(combined.end(), std::next(segment.begin()), segment.end());
    }
    return true;
  }

  bool BuildGoalPlan(std::vector<Point2> &points, double &finalYaw, std::string &error)
  {
    if (!hasGoal_) {
      error = "waiting for target_pose";
      return false;
    }
    const Point2 start = CurrentPosition();
    const Point2 goal = PosePoint(goal_);
    finalYaw = YawFromQuaternion(goal_.pose.orientation);
    return AppendSegment(start, goal, points, error);
  }

  bool BuildCurvePlan(std::vector<Point2> &points, double &finalYaw, std::string &error)
  {
    if (!hasCurve_ || referenceCurve_.poses.empty()) {
      error = "waiting for reference_curve";
      return false;
    }
    const Point2 start = CurrentPosition();
    std::size_t nearest = 0;
    double nearestDistance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < referenceCurve_.poses.size(); ++i) {
      const double distance = Distance(start, PosePoint(referenceCurve_.poses[i]));
      if (distance < nearestDistance) {
        nearestDistance = distance;
        nearest = i;
      }
    }

    Point2 anchor = start;
    points = {start};
    if (hotCurveReplacement_ && activePath_.poses.size() > 1 &&
        IsBlocked(anchor, PlanningInflation())) {
      std::size_t oldNearest = 0;
      double oldNearestDistance = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < activePath_.poses.size(); ++i) {
        const double distance = Distance(anchor, PosePoint(activePath_.poses[i]));
        if (distance < oldNearestDistance) {
          oldNearestDistance = distance;
          oldNearest = i;
        }
      }
      bool recoveredPlanningClearance = false;
      for (std::size_t i = oldNearest; i < activePath_.poses.size(); ++i) {
        const Point2 recoveryPoint = PosePoint(activePath_.poses[i]);
        if (Distance(anchor, recoveryPoint) < 0.002) {
          continue;
        }
        if (!SegmentFree(anchor, recoveryPoint, SafetyInflation())) {
          error = "cannot safely reconnect to active trajectory";
          return false;
        }
        points.push_back(recoveryPoint);
        anchor = recoveryPoint;
        if (!IsBlocked(anchor, PlanningInflation())) {
          recoveredPlanningClearance = true;
          break;
        }
      }
      if (!recoveredPlanningClearance) {
        error = "active trajectory cannot recover planning clearance";
        return false;
      }
      nearest = 0;
      nearestDistance = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < referenceCurve_.poses.size(); ++i) {
        const double distance = Distance(anchor, PosePoint(referenceCurve_.poses[i]));
        if (distance < nearestDistance) {
          nearestDistance = distance;
          nearest = i;
        }
      }
    }
    bool foundUsablePoint = false;
    for (std::size_t i = nearest; i < referenceCurve_.poses.size(); ++i) {
      const Point2 candidate = PosePoint(referenceCurve_.poses[i]);
      if (IsBlocked(candidate, PlanningInflation())) {
        continue;
      }
      if (Distance(anchor, candidate) < 0.005) {
        foundUsablePoint = true;
        anchor = candidate;
        continue;
      }
      if (!AppendSegment(anchor, candidate, points, error)) {
        return false;
      }
      foundUsablePoint = true;
      anchor = candidate;
    }
    const Point2 requestedEnd = PosePoint(referenceCurve_.poses.back());
    if (!foundUsablePoint || IsBlocked(requestedEnd, PlanningInflation()) ||
        Distance(anchor, requestedEnd) > 0.005) {
      error = "reference curve endpoint is blocked or unreachable";
      return false;
    }
    finalYaw = YawFromQuaternion(referenceCurve_.poses.back().pose.orientation);
    return true;
  }

  std::vector<Point2> SmoothAndResample(const std::vector<Point2> &input) const
  {
    if (input.size() < 2) {
      return input;
    }
    std::vector<Point2> cleaned;
    cleaned.reserve(input.size());
    for (const auto &point : input) {
      if (cleaned.empty() || Distance(cleaned.back(), point) > 0.001) {
        cleaned.push_back(point);
      }
    }
    if (cleaned.size() < 2) {
      return cleaned;
    }

    std::vector<Point2> smooth = cleaned;
    const int iterations = std::max(
      0, static_cast<int>(this->get_parameter("curve_smoothing_iterations").as_int()));
    const double weight = std::clamp(
      this->get_parameter("curve_smoothing_weight").as_double(), 0.05, 0.45);
    for (int iteration = 0; iteration < iterations; ++iteration) {
      std::vector<Point2> candidate;
      candidate.reserve(smooth.size() * 2);
      candidate.push_back(smooth.front());
      for (std::size_t i = 0; i + 1 < smooth.size(); ++i) {
        const Point2 q{
          (1.0 - weight) * smooth[i].x + weight * smooth[i + 1].x,
          (1.0 - weight) * smooth[i].y + weight * smooth[i + 1].y};
        const Point2 r{
          weight * smooth[i].x + (1.0 - weight) * smooth[i + 1].x,
          weight * smooth[i].y + (1.0 - weight) * smooth[i + 1].y};
        if (i > 0) {
          candidate.push_back(q);
        }
        if (i + 2 < smooth.size()) {
          candidate.push_back(r);
        }
      }
      candidate.push_back(smooth.back());
      bool collisionFree = true;
      for (std::size_t i = 1; i < candidate.size(); ++i) {
        if (!SegmentFree(candidate[i - 1], candidate[i], PlanningInflation())) {
          collisionFree = false;
          break;
        }
      }
      if (!collisionFree) {
        break;
      }
      smooth = std::move(candidate);
    }

    const double spacing = std::max(
      0.005, this->get_parameter("trajectory_sample_distance").as_double());
    std::vector<double> cumulative(smooth.size(), 0.0);
    for (std::size_t i = 1; i < smooth.size(); ++i) {
      cumulative[i] = cumulative[i - 1] + Distance(smooth[i - 1], smooth[i]);
    }
    const double totalLength = cumulative.back();
    if (totalLength < kEpsilon) {
      return {smooth.front()};
    }
    const std::size_t sampleCount = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(totalLength / spacing)));
    std::vector<Point2> sampled;
    sampled.reserve(sampleCount + 1);
    std::size_t segment = 1;
    for (std::size_t sample = 0; sample <= sampleCount; ++sample) {
      const double arc = totalLength * static_cast<double>(sample) / sampleCount;
      while (segment + 1 < cumulative.size() && cumulative[segment] < arc) {
        ++segment;
      }
      const double segmentLength = cumulative[segment] - cumulative[segment - 1];
      const double ratio = segmentLength > kEpsilon ?
        (arc - cumulative[segment - 1]) / segmentLength : 0.0;
      sampled.push_back({
        smooth[segment - 1].x + ratio * (smooth[segment].x - smooth[segment - 1].x),
        smooth[segment - 1].y + ratio * (smooth[segment].y - smooth[segment - 1].y)});
    }
    return sampled;
  }

  nav_msgs::msg::Path MakePath(const std::vector<Point2> &points, const double finalYaw)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = "odom";
    for (std::size_t i = 0; i < points.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = points[i].x;
      pose.pose.position.y = points[i].y;
      double yaw = finalYaw;
      if (i + 1 < points.size()) {
        yaw = std::atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x);
      }
      pose.pose.orientation = QuaternionFromYaw(yaw);
      path.poses.push_back(std::move(pose));
    }
    return path;
  }

  wcr_planning_msgs::msg::TimedTrajectory MakeTrajectory(
    const std::vector<Point2> &points, const double finalYaw)
  {
    wcr_planning_msgs::msg::TimedTrajectory trajectory;
    trajectory.header.stamp = this->now();
    trajectory.header.frame_id = "odom";
    if (points.empty()) {
      return trajectory;
    }
    const std::size_t count = points.size();
    std::vector<double> arc(count, 0.0);
    std::vector<double> curvature(count, 0.0);
    std::vector<double> speed(count, 0.0);
    for (std::size_t i = 1; i < count; ++i) {
      arc[i] = arc[i - 1] + Distance(points[i - 1], points[i]);
    }
    for (std::size_t i = 1; i + 1 < count; ++i) {
      const Point2 a = points[i - 1];
      const Point2 b = points[i];
      const Point2 c = points[i + 1];
      const double ab = Distance(a, b);
      const double bc = Distance(b, c);
      const double ac = Distance(a, c);
      const double denominator = ab * bc * ac;
      if (denominator > kEpsilon) {
        curvature[i] = 2.0 * ((b.x - a.x) * (c.y - a.y) -
          (b.y - a.y) * (c.x - a.x)) / denominator;
      }
    }
    if (count > 2) {
      curvature.front() = curvature[1];
      curvature.back() = curvature[count - 2];
    }

    const double maxSpeed = std::max(
      0.01, this->get_parameter("max_trajectory_speed").as_double());
    const double lateralAcceleration = std::max(
      0.01, this->get_parameter("max_lateral_acceleration").as_double());
    const double minimumCruise = std::clamp(
      this->get_parameter("minimum_cruise_speed").as_double(), 0.0, maxSpeed);
    for (std::size_t i = 0; i < count; ++i) {
      const double curveLimit = std::sqrt(
        lateralAcceleration / std::max(std::abs(curvature[i]), 1.0e-6));
      speed[i] = std::min(maxSpeed, curveLimit);
      if (speed[i] < minimumCruise && curveLimit >= minimumCruise) {
        speed[i] = minimumCruise;
      }
    }
    speed.front() = 0.0;
    speed.back() = 0.0;
    const double acceleration = std::max(
      0.01, this->get_parameter("max_acceleration").as_double());
    const double deceleration = std::max(
      0.01, this->get_parameter("max_deceleration").as_double());
    for (std::size_t i = 1; i < count; ++i) {
      const double ds = arc[i] - arc[i - 1];
      speed[i] = std::min(
        speed[i], std::sqrt(speed[i - 1] * speed[i - 1] + 2.0 * acceleration * ds));
    }
    for (std::size_t i = count - 1; i > 0; --i) {
      const double ds = arc[i] - arc[i - 1];
      speed[i - 1] = std::min(
        speed[i - 1], std::sqrt(speed[i] * speed[i] + 2.0 * deceleration * ds));
    }

    trajectory.points.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      wcr_planning_msgs::msg::TrajectoryPoint point;
      point.pose.position.x = points[i].x;
      point.pose.position.y = points[i].y;
      double yaw = finalYaw;
      if (i + 1 < count) {
        yaw = std::atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x);
      }
      point.pose.orientation = QuaternionFromYaw(yaw);
      point.arc_length = arc[i];
      point.curvature = curvature[i];
      point.speed = speed[i];
      trajectory.points.push_back(std::move(point));
    }
    return trajectory;
  }

  void RequestReplan(const std::string &reason)
  {
    const bool hotCurveReplacement =
      reason == "reference curve updated" && mode_ == kCurveMode && executing_;
    hotCurveReplacement_ = hotCurveReplacement;
    replanReason_ = reason;
    replanPending_ = true;
    PublishReached(false);
    if (!hotCurveReplacement) {
      CancelController();
      executing_ = false;
      hasPreviousExecutionPosition_ = false;
      sendTrajectoryPending_ = false;
    }
    if (hasOdometry_) {
      Replan();
    }
  }

  void Replan()
  {
    replanPending_ = false;
    if (hasOdometry_ && IsBlocked(CurrentPosition(), SafetyInflation())) {
      StopForSafetyViolation();
      return;
    }
    safetyViolation_ = false;
    std::vector<Point2> points;
    double finalYaw = 0.0;
    std::string error;
    const bool success = mode_ == kGoalMode ?
      BuildGoalPlan(points, finalYaw, error) : BuildCurvePlan(points, finalYaw, error);
    if (!success || points.empty()) {
      activePath_ = nav_msgs::msg::Path();
      activePath_.header.stamp = this->now();
      activePath_.header.frame_id = "odom";
      pathPublisher_->publish(activePath_);
      trajectoryPublisher_->publish(wcr_planning_msgs::msg::TimedTrajectory());
      CancelController();
      PublishStatus("planning_failed: " + error);
      hotCurveReplacement_ = false;
      return;
    }

    points = SmoothAndResample(points);
    if (points.size() < 2) {
      CancelController();
      PublishStatus("planning_failed: trajectory is too short");
      hotCurveReplacement_ = false;
      return;
    }
    activePath_ = MakePath(points, finalYaw);
    activeTrajectory_ = MakeTrajectory(points, finalYaw);
    pathPublisher_->publish(activePath_);
    currentWaypoint_ = activePath_.poses.size() - 1;
    executing_ = true;
    previousExecutionPosition_ = CurrentPosition();
    hasPreviousExecutionPosition_ = true;
    sendTrajectoryPending_ = true;
    PublishStatus(
      "executing_mode_" + std::to_string(mode_) + ": " +
      std::to_string(activePath_.poses.size()) + " trajectory samples; " + replanReason_);
    hotCurveReplacement_ = false;
  }

  void UpdateExecution()
  {
    if (!executing_ || !sendTrajectoryPending_) {
      return;
    }
    sendTrajectoryPending_ = false;
    activeTrajectory_.header.stamp = this->now();
    trajectoryPublisher_->publish(activeTrajectory_);
  }

  void StopForSafetyViolation()
  {
    if (safetyViolation_) {
      return;
    }
    safetyViolation_ = true;
    executing_ = false;
    replanPending_ = false;
    hasPreviousExecutionPosition_ = false;
    sendTrajectoryPending_ = false;
    CancelController();
    PublishReached(false);
    PublishStatus("safety_violation: robot entered obstacle exclusion zone");
  }

  void CancelController()
  {
    if (!controllerCancelPublisher_) {
      return;
    }
    std_msgs::msg::Bool cancel;
    cancel.data = true;
    controllerCancelPublisher_->publish(cancel);
  }

  void PublishReached(const bool value)
  {
    if (!reachedPublisher_) {
      return;
    }
    std_msgs::msg::Bool reached;
    reached.data = value;
    reachedPublisher_->publish(reached);
  }

  void PublishStatus(const std::string &text)
  {
    if (!statusPublisher_) {
      RCLCPP_INFO(this->get_logger(), "%s", text.c_str());
      return;
    }
    std_msgs::msg::String status;
    status.data = text;
    statusPublisher_->publish(status);
    RCLCPP_INFO(this->get_logger(), "%s", text.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr modeSubscription_;
  rclcpp::Subscription<wcr_planning_msgs::msg::ObstacleArray>::SharedPtr
    obstacleArraySubscription_;
  rclcpp::Subscription<wcr_planning_msgs::msg::ObstacleUpdate>::SharedPtr
    obstacleUpdateSubscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSubscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr curveSubscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reachedSubscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr controllerTargetPublisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr controllerCancelPublisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher_;
  rclcpp::Publisher<wcr_planning_msgs::msg::TimedTrajectory>::SharedPtr trajectoryPublisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPublisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reachedPublisher_;
  rclcpp::TimerBase::SharedPtr executionTimer_;

  std::unordered_map<std::string, wcr_planning_msgs::msg::Obstacle> obstacles_;
  geometry_msgs::msg::PoseStamped goal_;
  nav_msgs::msg::Path referenceCurve_;
  nav_msgs::msg::Odometry odometry_;
  nav_msgs::msg::Path activePath_;
  wcr_planning_msgs::msg::TimedTrajectory activeTrajectory_;
  Point2 previousExecutionPosition_;
  std::string replanReason_;
  std::uint8_t mode_{kGoalMode};
  std::size_t currentWaypoint_{0};
  bool hasGoal_{false};
  bool hasCurve_{false};
  bool hasOdometry_{false};
  bool hasPreviousExecutionPosition_{false};
  bool replanPending_{false};
  bool executing_{false};
  bool sendTrajectoryPending_{false};
  bool hotCurveReplacement_{false};
  bool safetyViolation_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OnlineTrajectoryPlanner>());
  rclcpp::shutdown();
  return 0;
}
