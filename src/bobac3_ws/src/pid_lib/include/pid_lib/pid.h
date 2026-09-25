#ifndef __REI_PID_H__
#define __REI_PID_H__
#include <pid_lib/rei_print.h>

#include <cmath>
#include <iostream>
#define DEBUG_OUT true
namespace rei_tools {
class ReiPID {
 public:
  ReiPID(float Kp, float Ki, float Kd) : kp(Kp), ki(Ki), kd(Kd) {
    p_out = i_out = d_out = 0.0f;
    last_err = prev_last_err = 0.0f;
    sum_output = integral_limit = integral_sum = output_max_limit = 0.0f;
  }
  void setP(float p) { kp = p; }
  void setI(float i) { ki = i; }
  void setD(float d) { kd = d; }
  void setIntegralLimit(float limit) { integral_limit = limit; }
  void setOutputLimit(float max_limit, float min_limit) {
    output_max_limit = max_limit;
    output_min_limit = min_limit;
  }
  float compute(float current_value, float target_value) {
    float cur_err = target_value - current_value;

    integral_sum += cur_err;
    if (integral_sum > integral_limit) integral_sum = integral_limit;
    if (integral_sum < (integral_limit * (-1.0f)))
      integral_sum = (integral_limit * (-1.0f));

    p_out = kp * cur_err;
    i_out = ki * integral_sum;
    d_out = kd * (cur_err - last_err);
    sum_output = p_out + i_out + d_out;

    prev_last_err = last_err;
    last_err = cur_err;

    // print();
    return computeLimit(sum_output);
  }
  float computeIncrease(float current_value, float target_value) {
    float cur_err = target_value - current_value;

    integral_sum += cur_err;
    if (integral_sum > integral_limit) integral_sum = integral_limit;
    if (integral_sum < (integral_limit * (-1.0f)))
      integral_sum = (integral_limit * (-1.0f));

    p_out = kp * (cur_err - last_err);
    i_out = ki * integral_sum;
    d_out = kd * (cur_err + prev_last_err - 2 * last_err);
    sum_output += p_out + i_out + d_out;

    prev_last_err = last_err;
    last_err = cur_err;
    // print();
    return computeLimit(sum_output);
  }
  void print() {
    PCOUT_DEBUG << "last_err: " << last_err
                << "prev_last_err: " << prev_last_err << std::endl;
    PCOUT_DEBUG << "kp: " << kp << ", ki: " << ki << ", kd: " << kp
                << std::endl;
    PCOUT_DEBUG << "integral_sum: " << integral_sum
                << ", integral_limit: " << integral_limit
                << ", output_max_limit: " << output_max_limit << std::endl;
  }
  void reset_integral() { integral_sum = 0.0; }
  float signf(float value) {
    if (value < 0.0) return -1.0;
    return 1.0;
  }

 private:
  ReiPID() {
    p_out = i_out = d_out = 0.0f;
    last_err = prev_last_err = 0.0f;
    sum_output = integral_limit = integral_sum = output_max_limit = 0.0f;
  };

  float computeLimit(float value) {
    if (fabs(value) > output_max_limit)
      value = signf(value) * output_max_limit;
    else if (fabs(value) < output_min_limit)
      value = signf(value) * output_min_limit;
    return value;
  }

 private:
  float p_out, i_out, d_out;
  float kp, ki, kd;
  float last_err, prev_last_err;
  float integral_sum, integral_limit, sum_output;
  float output_max_limit, output_min_limit;
};
}  // namespace rei_tools
#endif  //
