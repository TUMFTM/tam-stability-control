#pragma once
#include <algorithm>
#include <cstddef>
#include <complex>
#include <initializer_list>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <variant>
#include <vector>

#include "tum_helpers_cpp/numerical.hpp"
#include "tum_types_cpp/control.hpp"
namespace tam::helpers::control
{
tam::types::control::ControlConstraintPoint find_constraint_point(
  const tam::types::control::ControlConstraints & constraints, const float idx);
tam::types::control::TrajectoryPoint find_trajectory_point(
  const tam::types::control::Trajectory & trajectory, const float idx);
tam::types::control::AdditionalInfoPoint find_additional_info_point(
  const tam::types::control::AdditionalTrajectoryInfos & additional_info, const float idx);
double find_heading(const tam::types::control::Trajectory & trajectory, const float idx);
template <typename T>
class FirstOrderLowPass
{
private:
  T pole_;
  T old_output_;

public:
  FirstOrderLowPass() = default;
  FirstOrderLowPass(const T & initial_output, const T & pole_value);
  T step(const T & input);
  void set_tf_pole(const T & tf_pole);
  void set_old_output(const T & old_output);
};
template <typename T>
struct PIDFeedback
{
  T feedback;
  T feedback_p;
  T feedback_i;
  T feedback_d;
  T error_integrator;
};
template <typename T>
class PIDControl
{
public:
  PIDControl() = default;
  PIDControl(
    const T & tf_pole, const T & kp, const T & ki, const T & kd, const T & tS,
    const T & saturation_low, const T & saturation_high);
  template <typename U>
  PIDFeedback<T> step(
    const T & error, const U & update_pd, const U & use_pd, const U & update_i, const U & reset_i,
    const U & use_i)
  {
    // update transfer function with new error
    T tf_error = d_filter_.step(error * update_pd);

    PIDFeedback<T> fb;
    // P-Feedback
    fb.feedback_p = kp_ * tf_error * use_pd;
    // D-Feedback
    fb.feedback_d = kd_ * (tf_error - error_old_) / tS_ * use_pd;
    // update error buffer
    error_old_ = tf_error;

    // update integrator
    error_integrator_ *= !reset_i;  // Reset integrator
    T integrator_update = error_integrator_ + (ki_ * error * tS_) * update_i;

    error_integrator_ +=
      (integrator_update - error_integrator_) *
      (integrator_update < saturation_high_ && integrator_update > saturation_low_);
    fb.error_integrator = error_integrator_;
    // I-feedback
    fb.feedback_i = error_integrator_ * use_i;

    // add up feedback
    fb.feedback = fb.feedback_p + fb.feedback_i + fb.feedback_d;

    return fb;
  }
  template <typename U>
  PIDFeedback<T> step(
    const T & error_pd, const T & error_i, const U & update_pd, const U & use_pd,
    const U & update_i, const U & reset_i, const U & use_i)
  {
    // update transfer function with new error
    T tf_error = d_filter_.step(error_pd * update_pd);

    PIDFeedback<T> fb;
    // P-Feedback
    fb.feedback_p = kp_ * tf_error * use_pd;
    // D-Feedback
    fb.feedback_d = kd_ * (tf_error - error_old_) / tS_ * use_pd;
    // update error buffer
    error_old_ = tf_error;

    // update integrator
    error_integrator_ *= !reset_i;  // Reset integrator
    T integrator_update = error_integrator_ + (ki_ * error_i * tS_) * update_i;

    error_integrator_ +=
      (integrator_update - error_integrator_) *
      (integrator_update < saturation_high_ && integrator_update > saturation_low_);
    fb.error_integrator = error_integrator_;
    // I-feedback
    fb.feedback_i = error_integrator_ * use_i;

    // add up feedback
    fb.feedback = fb.feedback_p + fb.feedback_i + fb.feedback_d;

    return fb;
  }
  void set_params(
    const T & tf_pole, const T & kp, const T & ki, const T & kd, const T & tS,
    const T & saturation_low, const T & saturation_high);

private:
  T kp_{}, ki_{}, kd_{}, tS_{}, error_integrator_{}, error_old_{}, saturation_low_{},
    saturation_high_{};
  FirstOrderLowPass<T> d_filter_;
};
class DiscreteTransferFunction
{
private:
  // H(z) = Y(z)/U(z) = K*(b_0 + b_1 * z^(-1) + ... + b_n * z^(-n) ) / (a_0 + a_1 * z^(-1) + ... +
  // a_m * z^(-m)) equivalently: y(t) = 1/a_0 * ( K * ( b_0 * u(t) + b_1 * u(t-1) + ... + b_n *
  // u(t-n) ) - a_1 * y(t-1) - a_2 * y(t-2) - ... - a_m * y(t-m) )
  std::vector<double> numerator_coeffs_;    // b_0, b_1, ..., b_n
  std::vector<double> denominator_coeffs_;  // a_0, a_1, ..., a_m
  std::vector<double> input_buffer_;        // u(t), u(t-1), ... u(t-n)
  std::vector<double> output_buffer_;       // y(t-1), y(t-2), ... y(t-m)
  double gain;

public:
  DiscreteTransferFunction(
    std::vector<double> num_coeffs, std::vector<double> den_coeffs, double gain)
  : numerator_coeffs_(num_coeffs), denominator_coeffs_(den_coeffs), gain(gain)
  {
    input_buffer_.resize(numerator_coeffs_.size());
    output_buffer_.resize(denominator_coeffs_.size() - 1);
    std::fill(input_buffer_.begin(), input_buffer_.end(), 0);
    std::fill(output_buffer_.begin(), output_buffer_.end(), 0);
  }
  explicit DiscreteTransferFunction() {}
  double step(double input_value)
  {
    // update input buffer with new value
    std::rotate(input_buffer_.begin(), input_buffer_.end() - 1, input_buffer_.end());
    input_buffer_[0] = input_value;
    // input_buffer_ is now: u(t), u(t-1), ..., u(t-n)

    double output =
      1 / denominator_coeffs_[0] *
      (gain * std::inner_product(
                numerator_coeffs_.begin(), numerator_coeffs_.end(), input_buffer_.begin(), 0.0) -
       std::inner_product(
         denominator_coeffs_.begin() + 1, denominator_coeffs_.end(), output_buffer_.begin(), 0.0));

    // update output buffer
    std::rotate(output_buffer_.begin(), output_buffer_.end() - 1, output_buffer_.end());
    output_buffer_[0] = output;

    return output;
  }
};
// Gives a DiscreteTransferFunction object for a continuous-time parameter set including damping
// ratio and eigenfrequency continuous-time: G(s) = (eigenfrequency^2) (s^2 + 2 * damping *
// eigenfrequency * s + eigenfrequency^2)
// Source: https://www.dsprelated.com/showarticle/1341.php
inline DiscreteTransferFunction get_second_order_lowpass_discrete_tf(
  const double t_sampling_s, const double damping, const double eigenfrequency)
{
  // continuous-time poles
  std::complex<double> p1_cont =
    -damping * eigenfrequency +
    eigenfrequency * std::sqrt(std::complex<double>(std::pow(damping, 2) - 1));
  std::complex<double> p2_cont =
    -damping * eigenfrequency -
    eigenfrequency * std::sqrt(std::complex<double>(std::pow(damping, 2) - 1));
  // discrete-time poles
  std::complex<double> p1 = std::exp(p1_cont * t_sampling_s);
  std::complex<double> p2 = std::exp(p2_cont * t_sampling_s);

  // Coefficients for numerator (b)
  std::vector<double> b(3);
  b[0] = 1;
  b[1] = 2;
  b[2] = 1;

  // Coefficients for denominator (a)
  std::vector<double> a(3);
  a[0] = 1;
  a[1] = (-1.0 * (p1 + p2)).real();  // always real anyway
  a[2] = (p1 * p2).real();           // always real anyway

  double K = std::accumulate(a.begin(), a.end(), 0.0) / std::accumulate(b.begin(), b.end(), 0.0);

  std::vector<double> numerator_coeffs = b;
  std::vector<double> denominator_coeffs = a;

  DiscreteTransferFunction second_order_lowpass_tf(numerator_coeffs, denominator_coeffs, K);

  return second_order_lowpass_tf;
}
template <typename T>
class Cyclic_Vector
{
private:
  std::vector<T> data;
  std::size_t index = 0;

public:
  Cyclic_Vector() = default;
  Cyclic_Vector(int size) : data(std::vector<T>(size)) {}
  Cyclic_Vector(std::initializer_list<T> initializer_list)
  {
    for (auto & elem : initializer_list) {
      data.push_back(elem);
    }
  }
  Cyclic_Vector(int size, T initial_data)
  {
    data = std::vector<T>(size);
    std::fill(data.begin(), data.end(), initial_data);
  }
  void insert(T new_element)
  {
    if (data.empty()) {
      throw std::logic_error("Cannot insert into an empty Cyclic_Vector");
    }
    index = (index + 1) % data.size();
    data.at(index) = new_element;
  }
  std::vector<T>::iterator begin() { return data.begin(); }
  std::vector<T>::iterator end() { return data.end(); }
  int size() { return data.size(); }
  std::vector<T> get_data() { return data; }
  void resize(int count)
  {
    if (count <= 0) {
      data.clear();
      index = 0;
      return;
    }
    data.resize(static_cast<std::size_t>(count));
    index %= data.size();
  }
  int get_index() { return index; }
  // Position is relative to index
  T get_element(int position)
  {
    if (data.empty()) {
      throw std::logic_error("Cannot read from an empty Cyclic_Vector");
    }
    const auto size = static_cast<std::ptrdiff_t>(data.size());
    const auto offset =
      (static_cast<std::ptrdiff_t>(index) + static_cast<std::ptrdiff_t>(position)) % size;
    return data.at(static_cast<std::size_t>(offset < 0 ? offset + size : offset));
  }
};
template <typename T>
class Flank_Detector
{
private:
  T data{};
  double eps{1e-5};

public:
  Flank_Detector() = default;
  explicit Flank_Detector(double eps_) : eps(eps_) {}
  bool check_and_update(T new_data)
  {
    if (data != new_data) {
      data = new_data;
      return true;
    }
    return false;
  }
  bool check_and_update(double new_data)
  {
    if (abs(data - new_data) > eps) {
      data = new_data;
      return true;
    }
    return false;
  }
};
}  // namespace tam::helpers::control
#include "controller_helpers_cpp/helpers_impl.hpp"
