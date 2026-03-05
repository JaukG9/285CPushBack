#include "main.h"
#include "lemlib/api.hpp"
#include "drivetrain.h"

pros::MotorGroup left_mg({-3, -4, -5}, pros::MotorGearset::blue); // left motor group (clockwise --> omni = counterclockwise)
pros::MotorGroup right_mg({8, 9, 10}, pros::MotorGearset::blue); // right motor group (counterclockwise --> omni = clockwise)

/* creation of drivetrain */
lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              11.22, // 11.22 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450 (600 * (36 / 48))
                              8 // horizontal drift is 8 due to presence of center traction wheels
);