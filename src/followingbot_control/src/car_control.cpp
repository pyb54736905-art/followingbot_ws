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

    // ── 로봇 기구학 ────────────────────────────────────────────────
    wheelbase_       = declare_parameter<double>("wheelbase", 1.0);
    max_steer_deg_   = declare_parameter<double>("max_steer_deg", 28.0);

    // ── Stanley 파라미터 ───────────────────────────────────────────
    // stanley_k: cross-track error 게인. 클수록 적극적으로 경로 복귀
    stanley_k_       = declare_parameter<double>("stanley_k", 1.0);

    // ── Pure Pursuit 파라미터 ──────────────────────────────────────
    // pp_lookahead_idx: Pure Pursuit 전용 lookahead 스텝 수
    // (Stanley의 lookahead_idx 와 독립적으로 튜닝 가능)
    pp_lookahead_idx_ = declare_parameter<int>("pp_lookahead_idx", 3);

    // ── 횡방향 블렌딩 파라미터 ─────────────────────────────────────
    // use_speed_blend = true:  속도 기반 자동 블렌딩
    //   v < pp_blend_speed_low  → Pure Pursuit 100%
    //   v > pp_blend_speed_high → Stanley 100%
    //   그 사이                  → 선형 보간
    // use_speed_blend = false: stanley_weight 고정값 사용
    //   stanley_weight = 1.0 → 순수 Stanley
    //   stanley_weight = 0.0 → 순수 Pure Pursuit
    //   stanley_weight = 0.7 → Stanley 70% + Pure Pursuit 30%
    use_speed_blend_      = declare_parameter<bool>  ("use_speed_blend",      true);
    stanley_weight_       = declare_parameter<double>("stanley_weight",       0.7);
    pp_blend_speed_low_   = declare_parameter<double>("pp_blend_speed_low",   0.15);
    pp_blend_speed_high_  = declare_parameter<double>("pp_blend_speed_high",  0.35);

    // ── 종방향 파라미터 ────────────────────────────────────────────
    target_speed_          = declare_parameter<double>("target_speed",          0.0);
    use_dynamic_speed_     = declare_parameter<bool>  ("use_dynamic_speed",     true);
    dynamic_speed_timeout_ = declare_parameter<double>("dynamic_speed_timeout", 0.5);

    speed_kp_      = declare_parameter<double>("speed_kp",      1.0);
    speed_ki_      = declare_parameter<double>("speed_ki",      0.0);
    speed_kd_      = declare_parameter<double>("speed_kd",      0.0);
    accel_limit_   = declare_parameter<double>("accel_limit",   1.0);
    decel_limit_   = declare_parameter<double>("decel_limit",   1.5);
    max_speed_cmd_ = declare_parameter<double>("max_speed_cmd", 5.0);
    speed_deadband_= declare_parameter<double>("speed_deadband",0.03);
    allow_reverse_ = declare_parameter<bool>  ("allow_reverse", false);

    // ── 기타 ───────────────────────────────────────────────────────
    lookahead_idx_            = declare_parameter<int> ("lookahead_idx",            5);
    use_lowlevel_speed_feedback_ = declare_parameter<bool>("use_lowlevel_speed_feedback", true);

    // ── 초기화 ─────────────────────────────────────────────────────
    max_steer_rad_ = max_steer_deg_ * M_PI / 180.0;
    stanley_.setParams(stanley_k_, wheelbase_, max_steer_rad_);
    lon_.setParams(speed_kp_, speed_ki_, speed_kd_, -decel_limit_, accel_limit_);

    // ── 구독 ──────────────────────────────────────────────────────
    sub_desired_speed_ = create_subscription<std_msgs::msg::Float64>(
      "/desired_speed", 10,
      std::bind(&CarControlNode::onDesiredSpeed, this, std::placeholders::_1));
    sub_path_ = create_subscription<nav_msgs::msg::Path>(
      "/path", 10,
      std::bind(&CarControlNode::onPath, this, std::placeholders::_1));
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 50,
      std::bind(&CarControlNode::onOdom, this, std::placeholders::_1));
    measured_speed_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/measured_speed", 10,
      std::bind(&CarControlNode::onMeasuredSpeed, this, std::placeholders::_1));

    // ── 발행 ──────────────────────────────────────────────────────
    target_speed_pub_ = create_publisher<std_msgs::msg::Float64>("/target_speed",       10);
    target_steer_pub_ = create_publisher<std_msgs::msg::Float64>("/target_steer",       10);
    pub_cmd_vel_      = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",         10);
    // 튜닝용 진단 토픽
    pub_stanley_steer_ = create_publisher<std_msgs::msg::Float64>("/dbg_stanley_steer", 10);
    pub_pp_steer_      = create_publisher<std_msgs::msg::Float64>("/dbg_pp_steer",      10);
    pub_blend_weight_  = create_publisher<std_msgs::msg::Float64>("/dbg_blend_weight",  10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&CarControlNode::onTimer, this));

    last_time_              = now();
    last_desired_speed_time_= now();

    RCLCPP_INFO(get_logger(),
      "CarControlNode started | stanley_k=%.2f pp_lookahead_idx=%d "
      "use_speed_blend=%s stanley_weight=%.2f blend_speed=[%.2f, %.2f]",
      stanley_k_, pp_lookahead_idx_,
      use_speed_blend_ ? "true" : "false",
      stanley_weight_, pp_blend_speed_low_, pp_blend_speed_high_);
  }

private:
  // ── 콜백 ────────────────────────────────────────────────────────
  void onDesiredSpeed(const std_msgs::msg::Float64::SharedPtr msg) {
    desired_speed_ = msg->data;
    last_desired_speed_time_ = now();
    has_desired_speed_ = true;
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr msg) {
    path_     = *msg;
    has_path_ = !path_.poses.empty();
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_     = *msg;
    has_odom_ = true;
  }

  void onMeasuredSpeed(const std_msgs::msg::Float64::SharedPtr msg) {
    current_speed_mps_  = msg->data;
    has_measured_speed_ = true;
  }

  // ── 유틸리티 ─────────────────────────────────────────────────────
  int findNearestIndex(double x, double y) const {
    if (!has_path_) return -1;
    double best_d2 = std::numeric_limits<double>::infinity();
    int    best_i  = -1;
    for (size_t i = 0; i < path_.poses.size(); ++i) {
      const auto& p  = path_.poses[i].pose.position;
      const double dx = x - p.x, dy = y - p.y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) { best_d2 = d2; best_i = static_cast<int>(i); }
    }
    return best_i;
  }

  // ── 메인 제어 루프 ───────────────────────────────────────────────
  void onTimer() {
    if (!has_path_ || !has_odom_) return;

    const auto   t   = now();
    const double dt  = (t - last_time_).seconds();
    last_time_ = t;
    if (dt <= 0.0) return;

    // 로봇 자세
    const double x   = odom_.pose.pose.position.x;
    const double y   = odom_.pose.pose.position.y;
    const double yaw = yaw_from_quat(odom_.pose.pose.orientation);

    const double v_odom = odom_.twist.twist.linear.x;
    const double v_meas = (use_lowlevel_speed_feedback_ && has_measured_speed_)
                           ? current_speed_mps_ : v_odom;

    // 최근접 인덱스
    const int nearest = findNearestIndex(x, y);
    if (nearest < 0) return;

    // ── Stanley용 lookahead 포인트 ────────────────────────────────
    const int stanley_i = std::min(
      nearest + std::max(0, lookahead_idx_),
      static_cast<int>(path_.poses.size()) - 1);

    const auto& pt_s = path_.poses[stanley_i].pose.position;
    const auto& p0_s = path_.poses[std::max(0, stanley_i - 1)].pose.position;

    const double path_yaw = std::atan2(pt_s.y - p0_s.y, pt_s.x - p0_s.x);

    const double dx_s = x - pt_s.x, dy_s = y - pt_s.y;
    const double nx   = -std::sin(path_yaw), ny = std::cos(path_yaw);
    const double e_ct = dx_s * nx + dy_s * ny;

    // ── Stanley 조향 계산 ─────────────────────────────────────────
    stanley_.set_stanly_data(v_meas, path_yaw, yaw, e_ct);
    const double stanley_steer = stanley_.calc_stanly_steer();

    // ── Pure Pursuit용 lookahead 포인트 ───────────────────────────
    const int pp_i = std::min(
      nearest + std::max(0, pp_lookahead_idx_),
      static_cast<int>(path_.poses.size()) - 1);

    const auto& pt_p = path_.poses[pp_i].pose.position;

    const double dx_pp = pt_p.x - x;
    const double dy_pp = pt_p.y - y;
    const double ld    = std::hypot(dx_pp, dy_pp);

    // alpha: 로봇 heading 기준 lookahead 포인트 방향각
    const double alpha = normalize_angle(std::atan2(dy_pp, dx_pp) - yaw);

    // Pure Pursuit 조향각 (ld 최솟값 보호)
    const double pp_raw   = (ld > 0.05)
                             ? std::atan2(2.0 * wheelbase_ * std::sin(alpha), ld)
                             : 0.0;
    const double pp_steer = std::clamp(pp_raw, -max_steer_rad_, max_steer_rad_);

    // ── 횡방향 블렌딩 ─────────────────────────────────────────────
    double w_stanley;
    if (use_speed_blend_) {
      const double dv = std::max(pp_blend_speed_high_ - pp_blend_speed_low_, 1e-3);
      w_stanley = std::clamp((v_meas - pp_blend_speed_low_) / dv, 0.0, 1.0);
    } else {
      w_stanley = stanley_weight_;
    }

    const double steer_cmd = std::clamp(
      w_stanley * stanley_steer + (1.0 - w_stanley) * pp_steer,
      -max_steer_rad_, max_steer_rad_);

    // ── 종방향: 목표 속도 결정 ────────────────────────────────────
    if (use_dynamic_speed_ && has_desired_speed_) {
      const double age = (t - last_desired_speed_time_).seconds();
      target_speed_mps_ = (age < dynamic_speed_timeout_) ? desired_speed_ : target_speed_;
    } else {
      target_speed_mps_ = target_speed_;
    }

    speed_error_mps_ = target_speed_mps_ - v_meas;

    const double a_cmd = lon_.calc_acc_cmd(target_speed_mps_, v_meas, dt);

    v_cmd_ = v_cmd_ + a_cmd * dt;

    if (allow_reverse_) {
      v_cmd_ = std::clamp(v_cmd_, -max_speed_cmd_, max_speed_cmd_);
    } else {
      v_cmd_ = std::clamp(v_cmd_, 0.0, max_speed_cmd_);
    }

    if (std::fabs(target_speed_mps_) < speed_deadband_) v_cmd_ = 0.0;

    // ── 발행 ────────────────────────────────────────────────────
    {
      std_msgs::msg::Float64 m; m.data = steer_cmd; target_steer_pub_->publish(m);
    }
    {
      std_msgs::msg::Float64 m; m.data = v_cmd_; target_speed_pub_->publish(m);
    }
    {
      geometry_msgs::msg::Twist t;
      t.linear.x = v_cmd_; t.angular.z = 0.0;
      pub_cmd_vel_->publish(t);
    }
    // 튜닝용 진단 발행
    {
      std_msgs::msg::Float64 m; m.data = stanley_steer; pub_stanley_steer_->publish(m);
    }
    {
      std_msgs::msg::Float64 m; m.data = pp_steer; pub_pp_steer_->publish(m);
    }
    {
      std_msgs::msg::Float64 m; m.data = w_stanley; pub_blend_weight_->publish(m);
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "v=%.3f target=%.3f | stanley=%.3f pp=%.3f blend_w=%.2f steer=%.3f | e_ct=%.3f ld=%.2f",
      v_meas, target_speed_mps_,
      stanley_steer, pp_steer, w_stanley, steer_cmd,
      e_ct, ld);
  }

  // ── 파라미터 (횡방향) ─────────────────────────────────────────
  double wheelbase_{1.0};
  double max_steer_deg_{28.0};
  double max_steer_rad_{0.489};   // max_steer_deg_ 에서 변환, 멤버로 보관

  double stanley_k_{1.0};

  int    pp_lookahead_idx_{3};

  bool   use_speed_blend_{true};
  double stanley_weight_{0.7};
  double pp_blend_speed_low_{0.15};
  double pp_blend_speed_high_{0.35};

  // ── 파라미터 (종방향) ─────────────────────────────────────────
  double target_speed_{0.0};
  bool   use_dynamic_speed_{true};
  double dynamic_speed_timeout_{0.5};

  double speed_kp_{1.0};
  double speed_ki_{0.0};
  double speed_kd_{0.0};
  double accel_limit_{1.0};
  double decel_limit_{1.5};
  double max_speed_cmd_{5.0};
  double speed_deadband_{0.03};

  int  lookahead_idx_{5};
  bool use_lowlevel_speed_feedback_{true};
  bool allow_reverse_{false};

  // ── 상태 ──────────────────────────────────────────────────────
  nav_msgs::msg::Path      path_;
  nav_msgs::msg::Odometry  odom_;
  bool has_path_{false};
  bool has_odom_{false};
  bool has_measured_speed_{false};
  bool has_desired_speed_{false};

  double current_speed_mps_{0.0};
  double target_speed_mps_{0.0};
  double speed_error_mps_{0.0};
  double desired_speed_{0.0};
  double v_cmd_{0.0};

  rclcpp::Time last_time_;
  rclcpp::Time last_desired_speed_time_;

  // ── 제어기 ────────────────────────────────────────────────────
  Stanley    stanley_;
  LonControl lon_;

  // ── ROS 인터페이스 ────────────────────────────────────────────
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr  sub_desired_speed_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr     sub_path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr  measured_speed_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    target_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    target_steer_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

  // 튜닝용 진단 토픽
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    pub_stanley_steer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    pub_pp_steer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    pub_blend_weight_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarControlNode>());
  rclcpp::shutdown();
  return 0;
}