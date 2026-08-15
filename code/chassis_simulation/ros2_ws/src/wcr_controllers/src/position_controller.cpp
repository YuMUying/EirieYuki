#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "wcr_planning_msgs/msg/timed_trajectory.hpp"

namespace
{
constexpr double kEpsilon = 1.0e-9;

double NormalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double YawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
{
  const double sinyCosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosyCosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(sinyCosp, cosyCosp);
}

struct TrajectorySample
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double curvature{0.0};
  double speed{0.0};
};
}  // namespace

class PositionController final : public rclcpp::Node
{
public:
  PositionController()
  : Node("position_controller")
  {
    this->declare_parameter("target_pose_topic", "/wcr/target_pose");
    this->declare_parameter("trajectory_topic", "/wcr/planned_trajectory");
    this->declare_parameter("odom_topic", "/wcr/odom");
    this->declare_parameter("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter("cancel_topic", "/wcr/controller_cancel");
    this->declare_parameter("reached_topic", "/wcr/controller_target_reached");
    this->declare_parameter("control_rate", 50.0);
    this->declare_parameter("linear_gain", 1.0);
    this->declare_parameter("angular_gain", 1.5);
    this->declare_parameter("max_linear_speed", 0.15);
    this->declare_parameter("max_angular_speed", 0.6);
    this->declare_parameter("max_acceleration", 0.12);
    this->declare_parameter("max_deceleration", 0.16);
    this->declare_parameter("lookahead_distance", 0.045);
    this->declare_parameter("lookahead_time", 0.30);
    this->declare_parameter("max_position_correction_speed", 0.05);
    this->declare_parameter("position_tolerance", 0.015);
    this->declare_parameter("yaw_tolerance", 0.03);

    const auto targetTopic = this->get_parameter("target_pose_topic").as_string();
    const auto trajectoryTopic = this->get_parameter("trajectory_topic").as_string();
    const auto odomTopic = this->get_parameter("odom_topic").as_string();
    const auto commandTopic = this->get_parameter("cmd_vel_topic").as_string();
    const auto cancelTopic = this->get_parameter("cancel_topic").as_string();
    const auto reachedTopic = this->get_parameter("reached_topic").as_string();

    targetSubscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      targetTopic, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message)
      {
        if (!message->header.frame_id.empty() && message->header.frame_id != "odom") {
          RCLCPP_WARN(
            this->get_logger(),
            "Target frame [%s] is not odom; interpreting it in odom coordinates.",
            message->header.frame_id.c_str());
        }
        target_ = *message;
        trajectory_.points.clear();
        trajectoryActive_ = false;
        hasTarget_ = true;
        targetReached_ = false;
        commandedSpeed_ = 0.0;
      });

    trajectorySubscription_ =
      this->create_subscription<wcr_planning_msgs::msg::TimedTrajectory>(
      trajectoryTopic, rclcpp::QoS(10),
      [this](const wcr_planning_msgs::msg::TimedTrajectory::SharedPtr message)
      {
        if (!message->header.frame_id.empty() && message->header.frame_id != "odom") {
          RCLCPP_WARN(
            this->get_logger(),
            "Trajectory frame [%s] is not odom; rejecting trajectory.",
            message->header.frame_id.c_str());
          return;
        }
        if (message->points.size() < 2) {
          Stop(false);
          return;
        }
        for (std::size_t i = 1; i < message->points.size(); ++i) {
          if (message->points[i].arc_length + kEpsilon < message->points[i - 1].arc_length) {
            RCLCPP_WARN(this->get_logger(), "Rejecting trajectory with decreasing arc length.");
            Stop(false);
            return;
          }
        }
        trajectory_ = *message;
        const bool replacingActiveTrajectory = trajectoryActive_;
        trajectoryActive_ = true;
        hasTarget_ = false;
        targetReached_ = false;
        nearestSegment_ = 0;
        if (!replacingActiveTrajectory) {
          commandedSpeed_ = 0.0;
          hasLastControlStamp_ = false;
        }
        RCLCPP_INFO(
          this->get_logger(), "%s continuous trajectory with %zu samples.",
          replacingActiveTrajectory ? "Replacing" : "Tracking",
          trajectory_.points.size());
      });

    odomSubscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odomTopic, 20,
      [this](const nav_msgs::msg::Odometry::SharedPtr message)
      {
        odometry_ = *message;
        hasOdometry_ = true;
      });

    cancelSubscription_ = this->create_subscription<std_msgs::msg::Bool>(
      cancelTopic, 10,
      [this](const std_msgs::msg::Bool::SharedPtr message)
      {
        if (message->data) {
          Stop(false);
        }
      });

    commandPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>(commandTopic, 10);
    reachedPublisher_ = this->create_publisher<std_msgs::msg::Bool>(reachedTopic, 10);

    controlRate_ = std::max(1.0, this->get_parameter("control_rate").as_double());
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / controlRate_),
      std::bind(&PositionController::Update, this));
  }

private:
  void Stop(const bool reached)
  {
    trajectoryActive_ = false;
    hasTarget_ = false;
    targetReached_ = reached;
    commandedSpeed_ = 0.0;
    hasLastControlStamp_ = false;
    if (commandPublisher_) {
      commandPublisher_->publish(geometry_msgs::msg::Twist());
    }
    if (reachedPublisher_) {
      std_msgs::msg::Bool message;
      message.data = reached;
      reachedPublisher_->publish(message);
    }
  }

  double ProjectOntoTrajectory(const double x, const double y)
  {
    const auto &points = trajectory_.points;
    double bestDistanceSquared = std::numeric_limits<double>::infinity();
    double bestArc = points[nearestSegment_].arc_length;
    std::size_t bestSegment = nearestSegment_;
    const std::size_t first = nearestSegment_ > 2 ? nearestSegment_ - 2 : 0;
    for (std::size_t i = first; i + 1 < points.size(); ++i) {
      const double startX = points[i].pose.position.x;
      const double startY = points[i].pose.position.y;
      const double dx = points[i + 1].pose.position.x - startX;
      const double dy = points[i + 1].pose.position.y - startY;
      const double lengthSquared = dx * dx + dy * dy;
      double ratio = 0.0;
      if (lengthSquared > kEpsilon) {
        ratio = std::clamp(((x - startX) * dx + (y - startY) * dy) / lengthSquared, 0.0, 1.0);
      }
      const double projectedX = startX + ratio * dx;
      const double projectedY = startY + ratio * dy;
      const double distanceSquared =
        (x - projectedX) * (x - projectedX) + (y - projectedY) * (y - projectedY);
      if (distanceSquared < bestDistanceSquared) {
        bestDistanceSquared = distanceSquared;
        bestSegment = i;
        bestArc = points[i].arc_length +
          ratio * (points[i + 1].arc_length - points[i].arc_length);
      }
    }
    nearestSegment_ = std::max(nearestSegment_, bestSegment);
    return bestArc;
  }

  TrajectorySample InterpolateTrajectory(const double requestedArc) const
  {
    const auto &points = trajectory_.points;
    if (requestedArc <= points.front().arc_length) {
      const auto &point = points.front();
      return {point.pose.position.x, point.pose.position.y,
        YawFromQuaternion(point.pose.orientation), point.curvature, point.speed};
    }
    if (requestedArc >= points.back().arc_length) {
      const auto &point = points.back();
      return {point.pose.position.x, point.pose.position.y,
        YawFromQuaternion(point.pose.orientation), point.curvature, point.speed};
    }
    auto upper = std::lower_bound(
      points.begin(), points.end(), requestedArc,
      [](const auto &point, const double arc) {return point.arc_length < arc;});
    const auto &end = *upper;
    const auto &start = *std::prev(upper);
    const double interval = end.arc_length - start.arc_length;
    const double ratio = interval > kEpsilon ?
      (requestedArc - start.arc_length) / interval : 0.0;
    const double startYaw = YawFromQuaternion(start.pose.orientation);
    const double endYaw = YawFromQuaternion(end.pose.orientation);
    return {
      start.pose.position.x + ratio * (end.pose.position.x - start.pose.position.x),
      start.pose.position.y + ratio * (end.pose.position.y - start.pose.position.y),
      NormalizeAngle(startYaw + ratio * NormalizeAngle(endYaw - startYaw)),
      start.curvature + ratio * (end.curvature - start.curvature),
      start.speed + ratio * (end.speed - start.speed)};
  }

  void PublishReachedState()
  {
    std_msgs::msg::Bool reached;
    reached.data = targetReached_;
    reachedPublisher_->publish(reached);
  }

  void UpdateTrajectory()
  {
    const auto &current = odometry_.pose.pose;
    const double currentYaw = YawFromQuaternion(current.orientation);
    const auto &finalPoint = trajectory_.points.back();
    const double finalDx = finalPoint.pose.position.x - current.position.x;
    const double finalDy = finalPoint.pose.position.y - current.position.y;
    const double finalDistance = std::hypot(finalDx, finalDy);
    const double finalYaw = YawFromQuaternion(finalPoint.pose.orientation);
    const double finalYawError = NormalizeAngle(finalYaw - currentYaw);
    const double positionTolerance = this->get_parameter("position_tolerance").as_double();
    const double yawTolerance = this->get_parameter("yaw_tolerance").as_double();
    if (finalDistance <= positionTolerance && std::abs(finalYawError) <= yawTolerance) {
      if (!targetReached_) {
        RCLCPP_INFO(
          this->get_logger(), "Continuous trajectory reached at x=%.3f, y=%.3f, yaw=%.3f",
          current.position.x, current.position.y, currentYaw);
      }
      targetReached_ = true;
      trajectoryActive_ = false;
      commandedSpeed_ = 0.0;
      commandPublisher_->publish(geometry_msgs::msg::Twist());
      PublishReachedState();
      return;
    }

    const double currentArc = ProjectOntoTrajectory(current.position.x, current.position.y);
    const TrajectorySample progress = InterpolateTrajectory(currentArc);
    const double lookahead = std::max(
      0.005, this->get_parameter("lookahead_distance").as_double() +
      this->get_parameter("lookahead_time").as_double() * progress.speed);
    const TrajectorySample target = InterpolateTrajectory(currentArc + lookahead);
    const double speedProbeDistance = std::max(
      0.005, trajectory_.points[1].arc_length - trajectory_.points[0].arc_length);
    const TrajectorySample speedProbe = InterpolateTrajectory(currentArc + speedProbeDistance);
    const double desiredSpeed = std::max(progress.speed, speedProbe.speed);
    double dt = hasLastControlStamp_ ? 0.0 : 1.0 / controlRate_;
    const rclcpp::Time odomStamp(odometry_.header.stamp);
    if (hasLastControlStamp_ && odomStamp > lastControlStamp_) {
      dt = std::clamp((odomStamp - lastControlStamp_).seconds(), 0.001, 0.1);
    }
    lastControlStamp_ = odomStamp;
    hasLastControlStamp_ = true;
    const double acceleration = std::max(
      0.01, this->get_parameter("max_acceleration").as_double());
    const double deceleration = std::max(
      0.01, this->get_parameter("max_deceleration").as_double());
    commandedSpeed_ = std::clamp(
      desiredSpeed, commandedSpeed_ - deceleration * dt,
      commandedSpeed_ + acceleration * dt);

    const double linearGain = this->get_parameter("linear_gain").as_double();
    double correctionX = linearGain * (progress.x - current.position.x);
    double correctionY = linearGain * (progress.y - current.position.y);
    const double correctionMagnitude = std::hypot(correctionX, correctionY);
    const double maxCorrection = std::max(
      0.0, this->get_parameter("max_position_correction_speed").as_double());
    if (correctionMagnitude > maxCorrection && correctionMagnitude > kEpsilon) {
      correctionX *= maxCorrection / correctionMagnitude;
      correctionY *= maxCorrection / correctionMagnitude;
    }

    double worldVx = commandedSpeed_ * std::cos(target.yaw) + correctionX;
    double worldVy = commandedSpeed_ * std::sin(target.yaw) + correctionY;
    const double maxLinear = this->get_parameter("max_linear_speed").as_double();
    const double worldSpeed = std::hypot(worldVx, worldVy);
    if (worldSpeed > maxLinear && worldSpeed > kEpsilon) {
      worldVx *= maxLinear / worldSpeed;
      worldVy *= maxLinear / worldSpeed;
    }

    geometry_msgs::msg::Twist command;
    command.linear.x = std::cos(currentYaw) * worldVx + std::sin(currentYaw) * worldVy;
    command.linear.y = -std::sin(currentYaw) * worldVx + std::cos(currentYaw) * worldVy;
    command.angular.z = std::clamp(
      commandedSpeed_ * target.curvature +
      this->get_parameter("angular_gain").as_double() * NormalizeAngle(target.yaw - currentYaw),
      -this->get_parameter("max_angular_speed").as_double(),
      this->get_parameter("max_angular_speed").as_double());
    targetReached_ = false;
    commandPublisher_->publish(command);
    PublishReachedState();
  }

  void UpdatePointTarget()
  {
    const auto &current = odometry_.pose.pose;
    const double currentYaw = YawFromQuaternion(current.orientation);
    const double targetYaw = YawFromQuaternion(target_.pose.orientation);
    const double dx = target_.pose.position.x - current.position.x;
    const double dy = target_.pose.position.y - current.position.y;
    const double distance = std::hypot(dx, dy);
    const double yawError = NormalizeAngle(targetYaw - currentYaw);
    geometry_msgs::msg::Twist command;
    if (distance <= this->get_parameter("position_tolerance").as_double() &&
        std::abs(yawError) <= this->get_parameter("yaw_tolerance").as_double()) {
      targetReached_ = true;
    } else {
      const double linearGain = this->get_parameter("linear_gain").as_double();
      const double maxLinear = this->get_parameter("max_linear_speed").as_double();
      double vx = linearGain * (std::cos(currentYaw) * dx + std::sin(currentYaw) * dy);
      double vy = linearGain * (-std::sin(currentYaw) * dx + std::cos(currentYaw) * dy);
      const double requestedSpeed = std::hypot(vx, vy);
      if (requestedSpeed > maxLinear && requestedSpeed > kEpsilon) {
        vx *= maxLinear / requestedSpeed;
        vy *= maxLinear / requestedSpeed;
      }
      command.linear.x = vx;
      command.linear.y = vy;
      command.angular.z = std::clamp(
        this->get_parameter("angular_gain").as_double() * yawError,
        -this->get_parameter("max_angular_speed").as_double(),
        this->get_parameter("max_angular_speed").as_double());
      targetReached_ = false;
    }
    commandPublisher_->publish(command);
    PublishReachedState();
  }

  void Update()
  {
    if (!hasOdometry_) {
      return;
    }
    if (trajectoryActive_) {
      UpdateTrajectory();
    } else if (hasTarget_) {
      UpdatePointTarget();
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSubscription_;
  rclcpp::Subscription<wcr_planning_msgs::msg::TimedTrajectory>::SharedPtr trajectorySubscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancelSubscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commandPublisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reachedPublisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseStamped target_;
  wcr_planning_msgs::msg::TimedTrajectory trajectory_;
  nav_msgs::msg::Odometry odometry_;
  std::size_t nearestSegment_{0};
  double controlRate_{50.0};
  double commandedSpeed_{0.0};
  rclcpp::Time lastControlStamp_{0, 0, RCL_ROS_TIME};
  bool hasTarget_{false};
  bool hasOdometry_{false};
  bool hasLastControlStamp_{false};
  bool trajectoryActive_{false};
  bool targetReached_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionController>());
  rclcpp::shutdown();
  return 0;
}
