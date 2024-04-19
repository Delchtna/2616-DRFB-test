#include "main.h"

PID::PID() {
  reset_variables();
  set_constants(0, 0, 0, 0);
}

//
PID::PID(double p, double i, double d, double start_i) {
  reset_variables();
  set_constants(p, i, d, start_i);
}

//Setter and getter methods for PID constants
void PID::set_constants(double p, double i, double d, double p_start_i) {
  constants = { p, i, d, p_start_i };
}

PID::Constants PID::get_constants() { return constants; }

double PID::get_output() {
  std::lock_guard<pros::Mutex> guard(pid_mutex);
  return output;
}

//Setter and getter methods for target
void PID::set_target(double input) { 
  std::lock_guard<pros::Mutex> guard(pid_mutex);
  target = input; 
}
double PID::get_target() { 
  std::lock_guard<pros::Mutex> guard(pid_mutex);
  return target; 
  }

//Setter and getter methods for max_speed
void PID::set_max_speed(float speed) { 
  std::lock_guard<pros::Mutex> guard(pid_mutex);
  max_speed = speed; 
}
float PID::get_max_speed() { 
  std::lock_guard<pros::Mutex> guard(pid_mutex);
  return max_speed; 
}

double PID::get_error() { 
  std::lock_guard<pros::Mutex> guard(pid_mutex);
  return error; 
}


//Returns if PID object has terminated
bool PID::is_settled() {
  {
    std::lock_guard<pros::Mutex> guard(pid_mutex);
    if (fabs(error) < small_error) {
      //printf("Error: %.5f", error);
      time_settled += Util::DELAY_TIME;
    }
  }
  return (time_settled > exit_time);
}

//Set exit conditions of PID object
void PID::set_exit_conditions(float small_error, double exit_time) {
  this->small_error = small_error;
  this->exit_time = exit_time;
}

//Compute output of PID controller
double PID::compute(double current) {

  std::lock_guard<pros::Mutex> guard(pid_mutex);

  error = target - current;
  derivative = error - prev_error;

  if (constants.ki != 0) {
    if (fabs(error) < constants.start_i)
      integral += error;

    if (Util::sgn(error) != Util::sgn(prev_error))
      integral = 0;
  }

  output = (error * constants.kp) + (integral * constants.ki) + (derivative * constants.kd);
  
  prev_error = error;
  return output;
}

void PID::reset_variables() {
  std::lock_guard<pros::Mutex> guard(pid_mutex);
  
  time_settled = 0;
  max_speed = 0;
  output = 0;
  target = 0;
  error = 0;
  prev_error = 0;
  integral = 0;
}
