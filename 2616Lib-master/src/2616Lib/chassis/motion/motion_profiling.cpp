#include "main.h"

Motion_Profiling::Motion_Profiling() {}

void Motion_Profiling::set_trajectory(std::vector<Path_Point>& trajectory) {
  this->trajectory = trajectory;
  current_index = 0;
  starting_position = (chassis.left_tracker.get_value_inches() + chassis.right_tracker.get_value_inches()) / 2;
  left_wheel_sma.set_period(sma_period);
  right_wheel_sma.set_period(sma_period);
  at_end = false;
}

void Motion_Profiling::set_software_constants(double kP, double kV, double kA, double kS, double zeta, double beta, int sma_period) {
  this->kP = kP;
  this->kV = kV;
  this->kA = kA;
  this->kS = kS;
  this->zeta = zeta;
  this->beta = beta;
  this->sma_period = sma_period;
}

void Motion_Profiling::set_hardware_constants(double ratio, double track_width, double wheel_diameter) {
  this->ratio = ratio;
  this->track_width = track_width;
  this->wheel_diameter = wheel_diameter;
}


void Motion_Profiling::update_current_point() {
  current_point = trajectory.at(current_index);
}

void Motion_Profiling::run_ramsete_controller() {
  double desX = Util::inch_to_m(current_point.pose.x);
  double desY = Util::inch_to_m(current_point.pose.y);
  double des_theta = current_point.pose.angle;
  double des_vel = Util::inch_to_m(current_point.velocity);
  double des_angular_vel = current_point.angular_velocity;
   
  double x = Util::inch_to_m(chassis.get_pose().x);
  double y = Util::inch_to_m(chassis.get_pose().y);
  double theta = chassis.get_pose().angle;

  double global_ex = desX - x;
  double global_ey = desY - y;

  //X and Y formulas are swapped intentionally
  double ey = Util::x_rotate_point(global_ex, global_ey, theta, false);
  double ex = Util::y_rotate_point(global_ex, global_ey, theta, false);

  theta = M_PI / 2 - theta;
  des_theta = M_PI / 2 - des_theta;
  double et = Util::normalize(des_theta - theta);

  double k = 2 * zeta * sqrt(des_angular_vel * des_angular_vel + beta * des_vel * des_vel);
  double vel = des_vel * cos(et) + k * ex; //ey in odoprom coords
  double angular_vel = des_angular_vel + k * et + beta * des_vel * sin(et) * ey / et;
  
  
  vel = Util::m_to_inch(vel);
  delta_v = vel - Util::m_to_inch(des_vel);
  delta_w = angular_vel - des_angular_vel;

  current_point.velocity = vel;
  current_point.angular_velocity = angular_vel;
}

void Motion_Profiling::calculate_wheel_speeds() {
  double current_left_velocity = ((chassis.left_motors[0].get_actual_velocity()  * ratio) * wheel_diameter * M_PI) / 60;
  double current_right_velocity = ((chassis.right_motors[0].get_actual_velocity()  * ratio) * wheel_diameter * M_PI) / 60;
  double current_angular_velocity = (current_right_velocity - current_left_velocity) / track_width;
  left_wheel_sma.add_data(current_left_velocity);
  right_wheel_sma.add_data(current_right_velocity);
  current_left_velocity = left_wheel_sma.get_mean();
  current_right_velocity = right_wheel_sma.get_mean();

  double angular_error = current_point.angular_velocity - current_angular_velocity;
  left_wheel_velocity = current_point.velocity - (current_point.angular_velocity * track_width / 2);
  right_wheel_velocity = current_point.velocity + (current_point.angular_velocity * track_width / 2);

  double current_velocity = (current_right_velocity + current_left_velocity) / 2;
  double calculated_velocity = (right_wheel_velocity + left_wheel_velocity) / 2;
  current_point.acceleration = ((calculated_velocity * calculated_velocity - prev_velocity * prev_velocity) / (2 * prev_velocity * (Util::DELAY_TIME / 1000.0)));

  double left_error = left_wheel_velocity - current_left_velocity;
  double right_error = right_wheel_velocity - current_right_velocity;

  double current_position = (chassis.left_tracker.get_value_inches() + chassis.right_tracker.get_value_inches()) / 2;
  double delta_x = current_position - starting_position;
  double error = current_point.s - delta_x;

  double left_output = left_wheel_velocity * kV + current_point.acceleration * kA + left_error * kP + kS;
  double right_output = right_wheel_velocity * kV + current_point.acceleration * kA + right_error * kP + kS;

  prev_velocity = calculated_velocity;

  chassis.set_tank(left_output, right_output);
}

void Motion_Profiling::check_if_at_end() {
  if (current_index >= trajectory.size() - 1) {
    chassis.set_tank(0, 0);
    chassis.set_drive_mode(Chassis::e_drive_mode::STANDBY);
    at_end = true;
   } else {
    current_index += 1;
   }
}