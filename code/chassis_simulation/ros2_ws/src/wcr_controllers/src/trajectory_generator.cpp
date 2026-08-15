#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class TrajectoryPublisher : public rclcpp::Node
{
    public:
        TrajectoryPublisher(): Node("trajectory_publisher")
        {
        pub_ = this->create_publisher<nav_msgs::msg::Odometry>("wcr/desired_trajectory", 10);

        //parameters for selecting trajectory
        this->declare_parameter<std::string>("trajectory_type", "circle");
        this->get_parameter("trajectory_type", trajectory_type_);
        
        start_time_ = this->now();
        timer_ = this->create_wall_timer(10ms, std::bind(&TrajectoryPublisher::timer_callback, this));
    }
  private:
    void timer_callback()
    {
        double t = (this->now() - start_time_).seconds();
        double x_d = 0.0, y_d = 0.0, theta_d = 0.0;
        double v_x_d = 0.0, v_y_d = 0.0, omega_d = 0.0;
        double theta_d_unwrapped;

        if (trajectory_type_ == "circle")
        {
            double r = 0.25;
            double v = (2 * M_PI)/80;

            x_d     = r * sin(v * t);
            y_d     = r - r * cos(v * t);

            v_x_d   = r * v * cos(v * t);
            v_y_d   = r * v * sin(v * t);

            theta_d = atan2(v_y_d, v_x_d);  // same as v*t
            //theta_d_unwrapped = unwrapAngle(theta_d, theta_prev_, offset_);
            omega_d = v;
        }
        else if (trajectory_type_ == "eight")
        {
            double a = 0.5;      // half-width of "8"
            double b = 0.25;      // half-height of "8"
            double v = 0.1;      // angular frequency [rad/s]

            // Position
            x_d = a * sin(v * t);
            y_d = b * sin(v * t) * cos(v * t);

            v_x_d = a * v * cos(v * t);
            v_y_d= b * v * (cos(2 * v * t) - sin(2 * v * t)) / 2.0;
            theta_d = atan2(v_y_d, v_x_d);
            
            double v_x_dot = -a * v * v * sin(v * t);
            double v_y_dot = -b * v * v * (sin(2 * v * t) + cos(2 * v * t));
            omega_d = (v_x_d * v_y_dot - v_y_d * v_x_dot) /
              (v_x_d * v_x_d + v_y_d * v_y_d + 1e-9); // +epsilon to avoid div by 0
        }

        else if (trajectory_type_ == "line")
        {
          x_d = 0.05 * t;
          y_d = 0.0;

          v_x_d = 0.05;
          v_y_d = 0.0;

          theta_d = atan2(v_y_d, v_x_d);
          double v_x_dot = 0.0;
          double v_y_dot = 0.0;
          omega_d = 0.0;
        
        }

        nav_msgs::msg::Odometry msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";

        msg.pose.pose.position.x = x_d;
        msg.pose.pose.position.y = y_d;
        geometry_msgs::msg::Quaternion q;
        tf2::Quaternion tf_q;
        tf_q.setRPY(0.0, 0.0, theta_d);
        msg.pose.pose.orientation = tf2::toMsg(tf_q);

        // velocities are in base_link frame
        msg.twist.twist.linear.x = v_x_d * cos(theta_d) + v_y_d * sin(theta_d);
        msg.twist.twist.linear.y = -v_x_d * sin(theta_d) + v_y_d * cos(theta_d);
        msg.twist.twist.angular.z = omega_d;

        pub_->publish(msg);
    }

    double unwrapAngle(double theta, double &theta_prev, double &offset) {
      // Detect jump across -pi/pi
      double diff = theta - theta_prev;
      if (diff > M_PI) {
          offset -= 2.0 * M_PI;
      } else if (diff < -M_PI) {
          offset += 2.0 * M_PI;
      }

      theta_prev = theta;
      return theta + offset;
    }
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string trajectory_type_;
    rclcpp::Time start_time_;
    double theta_prev_ = 0.0;
    double offset_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}