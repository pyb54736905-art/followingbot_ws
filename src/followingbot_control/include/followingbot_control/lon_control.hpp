#pragma once
#include "followingbot_control/PIDController.hpp"

class LonControl {
public:
  void setParams(double kp, double ki, double kd, double out_min, double out_max) {
    pid_.setGains(kp, ki, kd);
    pid_.setLimits(out_min, out_max);
  }
  void reset() { pid_.reset(); }

  // 여기서는 “wheel_speed_cmd(m/s)”를 바로 만들기 위해
  // PID 출력은 [-a_max, +a_max] 같은 가속도 형태로 보고,
  // car_control.cpp에서 v_cmd = v_cmd + a_cmd*dt 로 적분하는 방식 사용.
  double calc_acc_cmd(double target_v, double current_v, double dt) {
    return pid_.update(target_v, current_v, dt);
  }

private:
  PIDController pid_;
};
