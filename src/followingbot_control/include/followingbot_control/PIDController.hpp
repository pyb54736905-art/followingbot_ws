#pragma once
#include <algorithm>
#include <cmath>

inline double clip(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

inline double normalize_angle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

inline double low_pass_filter(double prev, double curr, double alpha) {
  // alpha: 0~1, bigger => smoother
  return alpha * prev + (1.0 - alpha) * curr;
}

class PIDController {
public:
  void setGains(double kp, double ki, double kd) { kp_ = kp; ki_ = ki; kd_ = kd; }
  void setLimits(double umin, double umax) { umin_ = umin; umax_ = umax; }
  void reset() { integ_ = 0.0; prev_err_ = 0.0; }

  double update(double target, double meas, double dt) {
    if (dt <= 0.0) return 0.0;
    const double err = target - meas;
    integ_ += err * dt;
    const double deriv = (err - prev_err_) / dt;
    prev_err_ = err;

    double u = kp_ * err + ki_ * integ_ + kd_ * deriv;
    return clip(u, umin_, umax_);
  }

private:
  double kp_{0.0}, ki_{0.0}, kd_{0.0};
  double umin_{-1e9}, umax_{1e9};
  double integ_{0.0}, prev_err_{0.0};
};
