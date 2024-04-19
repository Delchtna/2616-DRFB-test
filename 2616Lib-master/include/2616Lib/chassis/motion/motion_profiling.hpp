#pragma once

#include "2616Lib/util/points/path_point.hpp"
#include "2616Lib/util/simple_moving_average.hpp"
#include <vector>

class Motion_Profiling {
  public:
    Motion_Profiling();
    Motion_Profiling(double kP, double kV, double kA);

    std::vector<Path_Point> trajectory;
    
    Path_Point current_point;
    int current_index;
    double left_wheel_velocity, right_wheel_velocity;
    double delta_v, delta_w;
    double acceleration;
    double prev_velocity;
    
    double starting_position;
    double kP, kV, kA, kS;
    double zeta, beta;
    double ratio, track_width, wheel_diameter;
    bool at_end;

    Simple_Moving_Average left_wheel_sma, right_wheel_sma;
    int sma_period = 5;

    //Methods
    void set_trajectory(std::vector<Path_Point>& trajectory);
    void update_current_point();
    void check_if_at_end();

    void set_software_constants(double kP, double kV, double kA, double kS, double zeta, double beta, int sma_period = 5);
    void set_hardware_constants(double ratio, double track_width, double wheel_diameter);

    void calculate_wheel_speeds();

    void run_ramsete_controller();
};

