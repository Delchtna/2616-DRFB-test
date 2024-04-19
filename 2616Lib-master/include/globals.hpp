/**********************************************************************************/
/*      Use this file to declare variables that are used in multiple files.       */
/*     For example, you should declare motors, sensors, and pneumatics here.      */
/*                                                                                */
/*       When declaring objects that physically connect to the brain, it's        */
/*       best to use #define to set up all the ports, which makes it easier       */
/*                   to keep track of and change all your ports.                  */
/*                                                                                */
/*     Methods can be declared here, but using separate .hpp files is better.     */
/**********************************************************************************/

#pragma once

#include "globals.hpp"
#include "pros/apix.h"
#include "2616Lib/subsystems/flywheel.hpp"
#include "2616Lib/subsystems/catapult.hpp"


/**************************************************************************************/
/*              All of the port names and numbers below are examples, so              */
/*                be sure to change them based on your specific robot!                */
/*                                                                                    */
/*       Motors and sensors that use Smart Ports (the wire with square ends) are      */
/*       defined using the NUMBER of the brain port that they are plugged into.       */
/*      Pneumatics and some other sensors that use 3-wire connectors are defined      */
/*             using the CHARACTER of the port that they are plugged into.            */
/*                                                                                    */
/* For motors and V5 rotation sensors, a negative number means the thing is reversed. */
/*                                                                                    */
/*  All motors on your robot's drive are set up using the chassis object in main.cpp  */
/**************************************************************************************/


// ************* Motor Ports *************
#define INTAKE_PORT 3
//You don't need to define both a motor and piston indexer, so remove what you aren't using
#define INDEXER_MOTOR_PORT -99
#define FLYWHEEL_PORT 99
#define CATAPULT_PORT 16


// ************* Sensor Ports *************
#define IMU_PORT_1 89
#define IMU_PORT_2 99
#define THREE_WIRE_EXPANDER_PORT 99
//Change these tracker ports based on the type of odom sensor you're using
#define PERPENDICULAR_TRACKER_PORT -8
#define PARALLEL_TRACKER_PORT -9
#define LEFT_TRACKER_PORT_TOP 'A'
#define LEFT_TRACKER_PORT_BOTTOM 'B'
#define CATAPULT_LIMIT_SWITCH_PORT 'H'


// ************* Piston Ports *************
#define INDEXER_PISTON_PORT 'G'
#define EXPANSION_PORT 'C'
#define ANGLE_CHANGER_PORT 'F'



// ************* Subsystem Objects *************
//If your robot has a flywheel, uncomment the `extern Flywheel flywheel; line below, the matching `Flywheel flywheel(...);` line in main.cpp below the Chassis constructor, and the `flywheel.set_pidf_constants(...)` line in main.cpp in `initialize()`.
extern Flywheel flywheel;
//If your robot has a catapult, uncomment BOTH this line and the matching line `Catapult catapult(...);` in main.cpp below the Chassis constructor.
// extern Catapult catapult;


// ************* Motor Objects *************
inline pros::Motor intake(INTAKE_PORT, pros::E_MOTOR_GEAR_600);
inline pros::Motor indexer_motor(INDEXER_MOTOR_PORT, pros::E_MOTOR_GEAR_600);


// ************* Piston Objects *************
inline pros::ADIDigitalOut indexer_piston(INDEXER_PISTON_PORT);
inline pros::ADIDigitalOut expansion_piston(EXPANSION_PORT);
inline pros::ADIDigitalOut angle_changer_piston(ANGLE_CHANGER_PORT);


// ************* Sensor Objects *************
inline pros::ADIDigitalIn auton_limit_switch_up({21, 'F'});
inline pros::ADIDigitalIn auton_limit_switch_down({21, 'E'});