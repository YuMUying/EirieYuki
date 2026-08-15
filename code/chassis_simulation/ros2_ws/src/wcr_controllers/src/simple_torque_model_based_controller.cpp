#include <cmath>
#include <unordered_map>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;
using Eigen::Matrix3d;
using Eigen::Vector3d;


class ReducedModelBasedControllerVelocity : public rclcpp::Node
{
public:
    ReducedModelBasedControllerVelocity()
    : Node("simple_torque_controller")
    {
        // sub       
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&ReducedModelBasedControllerVelocity::joint_states_callback, this, std::placeholders::_1));

        // pub
        driving_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/main_driving_controller/commands", 10);

        steering_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/main_steering_controller/commands", 10);

        desired_trajectory_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/desired_trajectory_v_dv", 10);

        output_trajectory_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/output_trajectory_v_dv", 10);

        tau_c_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/tau_command", 10);

        //timer
        last_t_ = this->get_clock()->now().seconds();
        simple_trajectory_timer_ = this->create_wall_timer(2ms, std::bind(&ReducedModelBasedControllerVelocity::simpleTrajectory, this));

        this->declare_parameter("m", 5190.0);
        this->declare_parameter("m_w", 32.03);
        this->declare_parameter("I_delta", 0.002);

        this->declare_parameter("Kp_1", 1.0);
        this->declare_parameter("Kp_2", 1.0);
        this->declare_parameter("Kp_3", 1.0);

        this->declare_parameter("Kd_1", 1.0);
        this->declare_parameter("Kd_2", 1.0);
        this->declare_parameter("Kd_3", 1.0);

        this->declare_parameter("Kp", 1.0);
        this->declare_parameter("Ki", 0.1);
        this->declare_parameter("B_ff", 0.2);
        this->declare_parameter("Kaw", 0.1);

        this->declare_parameter("v", 0.0);

        RCLCPP_INFO(this->get_logger(), "ReducedModelBasedControllerTorque node started");
    }

private:
    void simpleTrajectory(){
      //time
      double current_t = this->get_clock()->now().seconds();
      double dt =  current_t - last_t_;
      t_ += dt;
      last_t_ = current_t;

      double end_time = 2*M_PI*20;

      //velocity trajectory v
      //if (t_ > end_time){
      //  return;
      //}
      double v1_d =  sin(t_/10)/20;//0.05 - cos(t_/20)/20;
      double omega1_d = 0.0;//sin(t_/20)/400;
      double omega2_d = 0.0;//-sin(t_/20)/400;

      //acceleration trajectory dv/dt
      double dotv1_d = 0.0;//cos(t_/10)/200;
      double dotomega1_d = 0.0;//cos(t_/20)/8000;
      double dotomega2_d = 0.0;//-cos(t_/20)/8000;

      std_msgs::msg::Float64MultiArray input_d;
      input_d.data = {v1_d, omega1_d, omega2_d, dotv1_d, dotomega1_d, dotomega2_d};

      std_msgs::msg::Float64MultiArray output;
      output.data = {v_(0), v_(1), v_(2), dotv_(0), dotv_(1), dotv_(2)};

      double delta_f = delta_i_[0];
      double delta_r = delta_i_[1];
      double omega_f = dot_delta_i_[0];
      double omega_r = dot_delta_i_[1];

      Eigen::Vector3d v_d(v1_d, omega1_d, omega2_d);
      Eigen::Vector3d dotv_d(dotv1_d, dotomega1_d, dotomega2_d);

      //Vector3d tau = calculateTorques(delta_f, delta_r, omega_f, omega_r, v_, dotv_, v_d, dotv_d);
      //tau(0) = saturate(tau(0), -0.2, 0.2);
      //tau(1) = 0.0;//saturate(tau(1), -0.2, 0.2);
      //tau(2) = 0.0;//saturate(tau(2), -0.2, 0.2);

      double tau = PIController(v_d(0), v_(0), dt, 1.9, -1.9);

      std_msgs::msg::Float64MultiArray driving_torque_msg;
      std_msgs::msg::Float64MultiArray steering_torque_msg;

      driving_torque_msg.data = {tau};
      steering_torque_msg.data = {0.0, 0.0};

      driving_torque_pub_->publish(driving_torque_msg);
      steering_torque_pub_->publish(steering_torque_msg);
      desired_trajectory_pub_ ->publish(input_d);
      output_trajectory_pub_ ->publish(output);   
    }

    double PIController(double reference, double measurement, double dt, double u_max, double u_min)
    {
    double Kp   = this->get_parameter("Kp").as_double();
    double Ki   = this->get_parameter("Ki").as_double();
    double B_ff = this->get_parameter("B_ff").as_double();
    double Kaw  = this->get_parameter("Kaw").as_double();  // <-- new anti-windup gain (set ≈ Ki/Kp)

    double e = reference - measurement;

    // feed-forward
    double tau_ff = B_ff * reference;

    // PI (before integrator commit)
    double u_p = Kp * e;
    double u_i = integrator_term_;
    double u_unsat = tau_ff + u_p + u_i;

    // saturate
    double u = std::clamp(u_unsat, u_min, u_max);

    // back-calculation anti-windup
    integrator_term_ += (Ki * e + Kaw * (u - u_unsat)) * dt;

    return u;
}   



    Vector3d calculateTorques(
    double delta_f, double delta_r, double omega_f, double omega_r,
    const Vector3d& v, const Vector3d& dotv, const Vector3d& v_d, 
    const Vector3d& dotv_d)
    {
        // Robot parameters
        double r = 0.0254;
        double a = 0.1125;
        double b = 0.1125;
        double m = this->get_parameter("m").as_double() * 1e-3;
        double m_w = this->get_parameter("m_w").as_double() * 1e-3;
        double I_theta = m * ((2*a)*(2*a) + (2*b)*(2*b)) / 12.0;
        double I_w = 0.5 * m_w * r * r;
        double I_delta = this->get_parameter("I_delta").as_double();

        double I_b = I_theta + 4*m_w*(a*a + b*b);
        double S = a*a + b*b;
        double D = 4*S;

        // Precompute cos/sin
        double c1 = cos(delta_f), s1 = sin(delta_f);
        double c2 = cos(delta_r), s2 = sin(delta_r);

        // Wi
        double W1 = (-b*c1 + a*s1) / D;
        double W2 = (-b*c2 - a*s2) / D;
        double W3 = ( b*c2 - a*s2) / D;
        double W4 = ( b*c1 + a*s1) / D;

        double sumW = W1 + W2 + W3 + W4;
        double sumc = 2*(c1 + c2);
        double sums = 2*(s1 + s2);

        // Mass matrix M
        double M11 = I_b*sumW*sumW + (m/16.0)*(sumc*sumc + sums*sums) + 4*I_w/(r*r);
        Matrix3d M = Matrix3d::Zero();
        M(0,0) = M11;
        M(1,1) = 2*I_delta;
        M(2,2) = 2*I_delta;

        // Derivatives of Wi, ci, si
        double dotW1 = ( b*sin(delta_f) + a*cos(delta_f)) * omega_f / D;
        double dotW2 = ( b*sin(delta_r) - a*cos(delta_r)) * omega_r / D;
        double dotW3 = (-b*sin(delta_r) - a*cos(delta_r)) * omega_r / D;
        double dotW4 = (-b*sin(delta_f) + a*cos(delta_f)) * omega_f / D;

        double dotsumW = dotW1 + dotW2 + dotW3 + dotW4;
        double dotsumc = 2*(-s1*omega_f - s2*omega_r);
        double dotsums = 2*( c1*omega_f + c2*omega_r);

        // V matrix
        double V11 = I_b*(sumW*dotsumW) + (m/16.0)*(sumc*dotsumc + sums*dotsums);
        Matrix3d V = Matrix3d::Zero();
        V(0,0) = V11;

        // N matrix
        double sumxy = 2*a*(s1 - s2);
        double N11 = (1/r)*(sumW*sumxy + 0.25*(sumc*sumc + sums*sums) + 4);
        Matrix3d N = Matrix3d::Zero();
        N(0,0) = N11;
        N(1,1) = 2;
        N(2,2) = 2;

        // PD gains
        Matrix3d Kp = Matrix3d::Zero();
        Matrix3d Kd = Matrix3d::Zero();
        Kp.diagonal() << this->get_parameter("Kp_1").as_double(), this->get_parameter("Kp_2").as_double(), this->get_parameter("Kp_3").as_double();
        Kd.diagonal() << this->get_parameter("Kd_1").as_double(), this->get_parameter("Kd_2").as_double(), this->get_parameter("Kd_3").as_double();

        Vector3d tau = -Kp*(v - v_d) - Kd*(dotv - dotv_d) + N.colPivHouseholderQr().solve(M * dotv_d + V * v_d);
        tau(0) /= 4;
        tau(1) /= 2;
        tau(2) /= 2;
        return tau;
    }

    double saturate(double value, double min_val, double max_val) {
      if (value > max_val) return max_val;
      if (value < min_val) return min_val;
    return value;
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

    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
      if (joint_index_.empty()) {
          for (size_t i = 0; i < msg->name.size(); ++i) {
              joint_index_[msg->name[i]] = i;
          }
          last_js_msg_ = msg;
          return;
      }
      // compute dt
      rclcpp::Time current_time = msg->header.stamp;
      rclcpp::Time last_time = last_js_msg_->header.stamp;
      double dt = (current_time - last_time).seconds();

      delta_i_[0] = msg->position[joint_index_["FL_steering"]];
      delta_i_[1] = msg->position[joint_index_["BL_steering"]];
      delta_i_[2] = msg->position[joint_index_["BR_steering"]];
      delta_i_[3] = msg->position[joint_index_["FR_steering"]];

      dot_delta_i_[0] = msg->velocity[joint_index_["FL_steering"]];
      dot_delta_i_[1] = msg->velocity[joint_index_["BL_steering"]];
      dot_delta_i_[2] = msg->velocity[joint_index_["BR_steering"]];
      dot_delta_i_[3] = msg->velocity[joint_index_["FR_steering"]];

      v_(0) = msg->velocity[joint_index_["FL_wheel"]] * 0.0254; //pomozni s r
      v_(1) = msg->velocity[joint_index_["FL_steering"]];
      v_(2) = msg->velocity[joint_index_["BL_steering"]];

      dotv_(0) = ((msg->velocity[joint_index_["FL_wheel"]] - last_js_msg_->velocity[joint_index_["FL_wheel"]]) * 0.0254) / dt;
      dotv_(1) = (msg->velocity[joint_index_["FL_steering"]] - last_js_msg_->velocity[joint_index_["FL_steering"]]) / dt;
      dotv_(2) = (msg->velocity[joint_index_["BL_steering"]] - last_js_msg_->velocity[joint_index_["BL_steering"]]) / dt;

      last_js_msg_ = msg;


    }
    void optimiseSpeedAngle(double returnSpeedAngle[], double speed, double angle) const{ 
      if(angle > (M_PI/2)){
          returnSpeedAngle[0] = -1 * speed;
          returnSpeedAngle[1] = angle - M_PI;
      }
      else if (angle <= (-M_PI/2)){
          returnSpeedAngle[0] = -1 * speed;
          returnSpeedAngle[1] = angle + M_PI;
      }
      else{
          returnSpeedAngle[0] = speed;
          returnSpeedAngle[1] = angle; 
      }
    }

    

    // sub
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr desired_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

    // pub
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desired_trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_c_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr output_trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr driving_torque_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_torque_pub_;

    rclcpp::TimerBase::SharedPtr trajectory_timeout_timer_;
    rclcpp::TimerBase::SharedPtr simple_trajectory_timer_;

    //dynamics controller
    Eigen::Vector3d v_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d dotv_ = Eigen::Vector3d::Zero();

    sensor_msgs::msg::JointState::SharedPtr last_js_msg_;
    
    double last_omega_c_ = 0.0;
    double last_delta_f_c_ = 0.0;
    double last_delta_r_c_ = 0.0;


    // state variables
    double v_x_, v_y_, omega_;
    double Kp_x_, Kp_y_, Kp_th_, Kp_delta_;
    double delta_i_[4], dot_delta_i_[4];
    double last_delta_c_i_[4] = {0.0, 0.0, 0.0, 0.0};
    double last_time_traj_ = -1.0;
    double theta_prev_ = 0.0;
    double theta_offset_ = 0.0;

    double x_d_, y_d_, theta_d_;
    double last_x_d_, last_y_d_, last_theta_d_;
    rclcpp::Time last_trajectory_update_time_;

    bool recived_first_trajectory_data_ = false;

    std::unordered_map<std::string, size_t> joint_index_;

    double t_ = 0.0;
    double last_t_ = 0.0;

    double integrator_term_;
    double d_filt_ = 0.0;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReducedModelBasedControllerVelocity>());
    auto node = std::make_shared<ReducedModelBasedControllerVelocity>();
    return 0;
}
