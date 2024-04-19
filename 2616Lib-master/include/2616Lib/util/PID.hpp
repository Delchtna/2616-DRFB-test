#pragma once

#include "pros/rtos.hpp"

class PID {
  public:
    PID();
    PID(double p, double i, double d, double start_i);

    //Contains PID constants
    struct Constants {
      double kp;
      double ki;
      double kd;
      double start_i;
    };

    //Set exit conditions of PID object
    void set_exit_conditions(float small_error, double exit_time);
    
    //Returns if PID has terminated
    bool is_settled();
    
    //Setter and getter for PID constants
    void set_constants(double p, double i, double d, double p_start_i);
    Constants get_constants();

    //Setter and getter for target
    void set_target(double input);
    double get_target();

    //Setter and getter for max_speed
    void set_max_speed(float speed);
    float get_max_speed();
    
    //Getter for error
    double get_error();

    //Computes the motor power
    double compute(double current);
    
    void reset_variables();
    double get_output();

    float small_error;

    pros::Mutex pid_mutex;


  private:
    Constants constants;
    double output;
    double error;
    double target;
    double prev_error;
    float max_speed;

    
    double exit_time;
    int time_settled;
    double integral;
    double derivative;
};