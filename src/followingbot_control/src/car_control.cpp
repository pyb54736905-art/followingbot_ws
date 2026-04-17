#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "followingbot_control/lat_control.hpp"
#include "followingbot_control/lon_control.hpp"
#include "followingbot_control/PIDController.hpp"

static double yaw_from_quat(const geometry_msgs::msg::Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

class CarControlNode : public rclcpp::Node {
public:
  CarControlNode() : Node("car_control_node") {
    // --- params ---
    wheelbase_ = declare_parameter<double>("wheelbase", 1.0);
    stanley_k_ = declare_parameter<double>("stanley_k", 1.0);
    max_steer_deg_ = declare_parameter<double>("max_steer_deg", 28.0);
    target_speed_ = declare_parameter<double>("target_speed", 1.5);

    speed_kp_ = declare_parameter<double>("speed_kp", 1.0);
    speed_ki_ = declare_parameter<double>("speed_ki", 0.0);
    speed_kd_ = declare_parameter<double>("speed_kd", 0.0);

    accel_limit_ = declare_parameter<double>("accel_limit", 1.0);
    decel_limit_ = declare_parameter<double>("decel_limit", 1.5);

    lookahead_idx_ = declare_parameter<int>("lookahead_idx", 5);
    use_lowlevel_speed_feedback_ =
      declare_parameter<bool>("use_lowlevel_speed_feedback", true);

    max_speed_cmd_ = declare_parameter<double>("max_speed_cmd", 5.0);
    speed_deadband_ = declare_parameter<double>("speed_deadband", 0.03);
    allow_reverse_ = declare_parameter<bool>("allow_reverse", false);

    const double max_steer_rad = (max_steer_deg_ * M_PI / 180.0);
    stanley_.setParams(stanley_k_, wheelbase_, max_steer_rad);
    lon_.setParams(speed_kp_, speed_ki_, speed_kd_, -decel_limit_, accel_limit_);

    sub_path_ = create_subscription<nav_msgs::msg::Path>(
      "/path", 10, std::bind(&CarControlNode::onPath, this, std::placeholders::_1));

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 50, std::bind(&CarControlNode::onOdom, this, std::placeholders::_1));

    measured_speed_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/measured_speed", 10,
      std::bind(&CarControlNode::onMeasuredSpeed, this, std::placeholders::_1));

    target_speed_pub_ = create_publisher<std_msgs::msg::Float64>("/target_speed", 10);
    target_steer_pub_ = create_publisher<std_msgs::msg::Float64>("/target_steer", 10);
    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&CarControlNode::onTimer, this));

    last_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "CarControlNode started. Inputs: /path, /odom, /measured_speed. Outputs: /target_speed, /target_steer, /cmd_vel");

    RCLCPP_INFO(
      get_logger(),
      "Params: wheelbase=%.3f, stanley_k=%.3f, max_steer_deg=%.3f, target_speed=%.3f, "
      "speed_kp=%.3f, speed_ki=%.3f, speed_kd=%.3f, accel_limit=%.3f, decel_limit=%.3f, "
      "max_speed_cmd=%.3f, speed_deadband=%.3f, lookahead_idx=%d, lowlevel_speed=%s, allow_reverse=%s",
      wheelbase_,
      stanley_k_,
      max_steer_deg_,
      target_speed_,
      speed_kp_,
      speed_ki_,
      speed_kd_,
      accel_limit_,
      decel_limit_,
      max_speed_cmd_,
      speed_deadband_,
      lookahead_idx_,
      use_lowlevel_speed_feedback_ ? "true" : "false",
      allow_reverse_ ? "true" : "false");
  }

private:
  void onPath(const nav_msgs::msg::Path::SharedPtr msg) {
    path_ = *msg;
    has_path_ = !path_.poses.empty();
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_ = *msg;
    has_odom_ = true;
  }

  void onMeasuredSpeed(const std_msgs::msg::Float64::SharedPtr msg) {
    current_speed_mps_ = msg->data;
    has_measured_speed_ = true;
  }

  int findNearestIndex(double x, double y) const {
    if (!has_path_) return -1;

    double best_d2 = std::numeric_limits<double>::infinity();
    int best_i = -1;

    for (size_t i = 0; i < path_.poses.size(); ++i) {
      const auto& p = path_.poses[i].pose.position;
      const double dx = x - p.x;
      const double dy = y - p.y;
      const double d2 = dx * dx + dy * dy;

      if (d2 < best_d2) {
        best_d2 = d2;
        best_i = static_cast<int>(i);
      }
    }
    return best_i;
  }

  void onTimer() {
    if (!has_path_ || !has_odom_) return;

    const auto t = now();
    const double dt = (t - last_time_).seconds();
    last_time_ = t;
    if (dt <= 0.0) return;

    // ego state from odom
    const double x = odom_.pose.pose.position.x;
    const double y = odom_.pose.pose.position.y;
    const double yaw = yaw_from_quat(odom_.pose.pose.orientation);

    const double v_odom = odom_.twist.twist.linear.x;
    const double v_meas =
      (use_lowlevel_speed_feedback_ && has_measured_speed_)
        ? current_speed_mps_
        : v_odom;

    // nearest + lookahead target
    const int nearest = findNearestIndex(x, y);
    if (nearest < 0) return;

    const int target_i = std::min(
      nearest + std::max(0, lookahead_idx_),
      static_cast<int>(path_.poses.size()) - 1);

    const auto& pt = path_.poses[target_i].pose.position;
    const auto& p0 = path_.poses[std::max(0, target_i - 1)].pose.position;

    // target heading from path segment
    const double path_yaw = std::atan2(pt.y - p0.y, pt.x - p0.x);

    // cross track error sign
    const double dx = x - pt.x;
    const double dy = y - pt.y;
    const double nx = -std::sin(path_yaw);
    const double ny = std::cos(path_yaw);
    const double e_ct = dx * nx + dy * ny;

    // --- lateral: stanley ---
    stanley_.set_stanly_data(v_meas, path_yaw, yaw, e_ct);
    const double steer_cmd = stanley_.calc_stanly_steer();

    // --- longitudinal: PID speed ---
    target_speed_mps_ = target_speed_;
    speed_error_mps_ = target_speed_mps_ - v_meas;

    const double a_cmd = lon_.calc_acc_cmd(target_speed_mps_, v_meas, dt);

    // 적분형 속도 명령 생성
    v_cmd_ = v_cmd_ + a_cmd * dt;

    // reverse 허용 여부에 따라 clamp 범위 분기
    if (allow_reverse_) {
      v_cmd_ = std::clamp(v_cmd_, -max_speed_cmd_, max_speed_cmd_);
    } else {
      v_cmd_ = std::clamp(v_cmd_, 0.0, max_speed_cmd_);
    }

    // 목표 속도가 거의 0이면 명령도 0으로 정리
    if (std::fabs(target_speed_mps_) < speed_deadband_) {
      v_cmd_ = 0.0;
    }

    // 아주 작은 속도 명령은 deadband로 0 처리
    if (std::fabs(v_cmd_) < speed_deadband_) {
      v_cmd_ = 0.0;
    }

    // publish steer
    std_msgs::msg::Float64 steer_msg;
    steer_msg.data = steer_cmd;
    target_steer_pub_->publish(steer_msg);

    // publish target speed
    std_msgs::msg::Float64 speed_msg;
    speed_msg.data = target_speed_mps_;
    target_speed_pub_->publish(speed_msg);

    // publish cmd_vel
    geometry_msgs::msg::Twist twist;
    twist.linear.x = v_cmd_;
    twist.angular.z = 0.0;
    pub_cmd_vel_->publish(twist);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "target=%.3f, measured=%.3f, error=%.3f, a_cmd=%.3f, v_cmd=%.3f, odom=%.3f, steer=%.3f, lowlevel=%s",
      target_speed_mps_,
      v_meas,
      speed_error_mps_,
      a_cmd,
      v_cmd_,
      v_odom,
      steer_cmd,
      (use_lowlevel_speed_feedback_ && has_measured_speed_) ? "true" : "false");
  }

  // params
  double wheelbase_{1.0};
  double stanley_k_{1.0};
  double max_steer_deg_{28.0};
  double target_speed_{1.5};

  double speed_kp_{1.0};
  double speed_ki_{0.0};
  double speed_kd_{0.0};

  double accel_limit_{1.0};
  double decel_limit_{1.5};
  double max_speed_cmd_{5.0};
  double speed_deadband_{0.03};

  int lookahead_idx_{5};

  bool use_lowlevel_speed_feedback_{true};
  bool allow_reverse_{false};

  // state
  nav_msgs::msg::Path path_;
  nav_msgs::msg::Odometry odom_;
  bool has_path_{false};
  bool has_odom_{false};
  bool has_measured_speed_{false};

  double current_speed_mps_{0.0};
  double target_speed_mps_{0.0};
  double speed_error_mps_{0.0};
  double v_cmd_{0.0};

  rclcpp::Time last_time_;

  // controllers
  Stanley stanley_;
  LonControl lon_;

  // ros
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr measured_speed_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_steer_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarControlNode>());
  rclcpp::shutdown();
  return 0;
}