#include "main.h"
#include "lemlib/api.hpp"
#include "robot/drivetrain.h"

/* creation of drivetrain motor groups */
pros::MotorGroup left_mg({-3, -4, -5}, pros::MotorGearset::blue);   // left motor group (motors spin clockwise --> wheels spin counterclockwise)
pros::MotorGroup right_mg({8, 9, 10}, pros::MotorGearset::blue);    // right motor group (motors spin counterclockwise --> wheels spin clockwise)

/* creation of drivetrain */
lemlib::Drivetrain drivetrain(&left_mg,                     // left motor group
                              &right_mg,                    // right motor group
                              11.22,                        // 11.22 inch track width
                              lemlib::Omniwheel::NEW_325,   // using new 3.25" omnis
                              450,                          // drivetrain rpm (450 = 600rpm * (36/48 gear ratio))
                              8                             // horizontal drift (8 because of center traction wheels)
);