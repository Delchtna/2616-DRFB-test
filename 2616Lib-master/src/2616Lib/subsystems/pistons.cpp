#include "main.h"

bool expansion_state = false;
//Set the state of the expansion piston to a new value
void set_expansion(bool state) {
  expansion_piston.set_value(state);
  expansion_state = state;
}

//Toggle the state of the expansion piston
void toggle_expansion() {
  set_expansion(!expansion_state);
}


bool angle_changer_state = false;
//Set the state of the angle changer to a new value
void set_angle_changer(bool state) {
  angle_changer_piston.set_value(state);
  angle_changer_state = state;
}

//Toggle the state of the angle changer
void toggle_angle_changer() {
  set_expansion(!angle_changer_state);
}