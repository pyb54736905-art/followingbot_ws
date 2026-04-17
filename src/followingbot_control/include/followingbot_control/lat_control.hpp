#pragma once
#include <cmath>
#include "followingbot_control/PIDController.hpp"

class Stanley {
public:
  void setParams(double k, double wheelbase, double max_steer_rad) {
    k_ = k;
    L_ = wheelbase;
    max_steer_ = max_steer_rad;
  }

  // 입력 세팅(너희 원본 구조를 따르는 형태)
  void set_stanly_data(double speed, double target_heading, double ego_yaw, double cross_track_error) {
    v_ = speed;
    target_heading_ = target_heading;
    ego_yaw_ = ego_yaw;
    e_ct_ = cross_track_error;
  }

  double calc_stanly_steer() const {
    // heading error
    const double heading_err = normalize_angle(target_heading_ - ego_yaw_);

    // crosstrack term (v near 0 protection)
    const double v = std::max(std::abs(v_), 0.1);
    const double crosstrack_term = std::atan2(k_ * e_ct_, v);

    double steer = heading_err + crosstrack_term;
    steer = normalize_angle(steer);
    steer = clip(steer, -max_steer_, max_steer_);
    return steer;
  }

private:
  double k_{1.0};
  double L_{1.0};
  double max_steer_{0.4886921906}; // 28deg rad

  double v_{0.0};
  double target_heading_{0.0};
  double ego_yaw_{0.0};
  double e_ct_{0.0};
};
