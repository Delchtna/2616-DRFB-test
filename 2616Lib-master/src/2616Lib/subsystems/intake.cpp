#include "main.h"

//todo make intake / indexer / pistons into classes?

//Spin the intake at a certain voltage. Voltage range is [-12000, 12000], and 0 stops the motor.
void set_intake(int voltage) {
  if (voltage == 0) {
    intake.move_velocity(0);
  } else {
    intake.move_voltage(voltage);
  }
}

//Spin the intake at a certain voltage for `millis` milliseconds, then stop
void intake_timed(int voltage, long millis) {
  set_intake(voltage);
  pros::delay(millis);
  set_intake(0);
}

//Check for forward or reverse intake buttons being held, and stop moving otherwise
void control_intake() {
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    set_intake(12000);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
    set_intake(-12000);
  } else {
    set_intake(0);
  }
}