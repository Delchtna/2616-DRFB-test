#include "main.h"

void set_DRFB(int voltage){
 if(voltage = 0){
    DRFB1.move_velocity(0);
    DRFB2.move_velocity(0);
 } else{
    DRFB1.move_voltage(-voltage);
    DRFB2.move_voltage(voltage);
 }
}

void control_DRFB(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    set_DRFB(12000);
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
    set_intake(-12000);
  } else {
    set_DRFB(0);
}
}