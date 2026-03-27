#include "main.h"
#include "lemlib/api.hpp"
#include "robot/odometry.h"
#include "robot/functions.h"
#include "robot/autos.h"

/**
 * @brief runs the skip autonomous.
 *
 * Moves the robot forward 6 inches while our alliance is running a solo AWP
 * autonomous, such that the robot is not touching the parking zone by the end
 * of the autonomous phase.
 */
void skip(){
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 6, 1000);    // move off of parking zone
}

/**
 * @brief runs the left side 4 block rush autonomous.
 *
 * Intakes the group of 3 blocks that start on left side of the field and outtakes
 * them, with the preload, into the long goal for a total of 4 blocks in the left
 * long goal. Then, wings the blocks into control.
 */
void left_4Rush(){
    chassis.setPose(-50.715, 4.034, 0); /*first position keep exact*/
    conveyorControl(600);
    chassis.chainToHeading(60, 750);
    chassis.chainToPoint(-22.5, 22, 1500);
    conveyorBrake();
    chassis.chainToHeading(327, 750);
    chassis.chainToPoint(-41, 48, 1500);
    conveyorBrake();
    chassis.chainToHeading(90, 750);
}

/**
 * @brief runs the right side 4 block rush autonomous.
 *
 * Intakes the group of 3 blocks that start on right side of the field and outtakes
 * them, with the preload, into the long goal for a total of 4 blocks in the right
 * long goal. Then, wings the blocks into control.
 */
void right_4Rush(){
    // intake the 3-block stack
    chassis.setPose(-47, -17.81, 105);
    conveyorControl(600);
    chassis.chainToPoint(-14.86, -24.39, 1500);

    // move to align with long goal
    chassis.chainToHeading(235, 750);
    chassis.moveToPoint(-48, -48, 1500);
    chassis.chainToHeading(270, 750);
    chassis.moveToPoint(-20, -48, 1200, {.forwards = false});
    chassis.waitUntilDone();

    // back into long goal while extaking
    chassis.moveToPoint(-15, -48, 2000, {.forwards = false, .maxSpeed = 30});
    pto.set_value(true);
    chassis.waitUntilDone();

    // wing blocks into control
    chassis.chainToPoint(-42, -48, 1000);
    pto.set_value(false);
    wing.set_value(true);
    chassis.chainToHeading(240, 750);
    chassis.chainToPoint(-25, -35, 1500, {.forwards = false});
    chassis.turnToHeading(270, 750);
    wing.set_value(false);
    while(true){
        chassis.moveToPoint(-4, -35, 200, {.forwards = false});
        chassis.waitUntilDone();
    }
}

/**
 * @brief runs the left side 7 block rush autonomous.
 *
 *
 *
 *
 *
 *
 */
void left_7Rush(){
    // intake the 3-block stack
    chassis.setPose(-55,8,62);
    conveyorControl(600);
    chassis.chainToPoint(-23, 22, 1500);
    chassis.chainToHeading(320, 750);

    // move to align with long goal intaking only 3 blocks
    chassis.chainToPoint(-47, 47, 1500);
    chassis.chainToHeading(270, 750);
    chassis.waitUntilDone();
    scraper.set_value(true);
    chassis.chainToPoint(-100, 47, 300, {.maxSpeed = 40});

    // back into long goal while extaking
    chassis.moveToPoint(-30, 47, 1500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(-15, 47, 2000, {.forwards = false, .maxSpeed = 30});
    pto.set_value(true);
    chassis.waitUntilDone();

    // wing blocks into control
    pto.set_value(false);
    wing.set_value(true);
    chassis.moveToPoint(-30, 47, 1500, {.forwards = false});
    chassis.chainToHeading(0, 750);
    chassis.chainToPoint(-30, 58, 1500);
    chassis.chainToHeading(270, 750);
    chassis.chainToPoint(-10, 58, 1500, {.forwards = false});
    wing.set_value(false);
    while(true){
        chassis.moveToPoint(-4, 58, 200, {.forwards = false});
        chassis.waitUntilDone();
    }
}

/**
 * @brief runs the right side 7 block rush autonomous.
 *
 *
 *
 *
 *
 *
 */
void right_7Rush(){
    // intake the 3-block stack
    chassis.setPose(-47, -12, 110);
    conveyorControl(600);
    chassis.chainToPoint(-22, -22, 1500);
    chassis.chainToHeading(222, 750);

    // move to align with long goal omt
    chassis.chainToPoint(-48, -47, 1500);
    chassis.chainToHeading(270, 750);
    scraper.set_value(true);
    chassis.moveToPoint(-99, -47, 300, {.maxSpeed = 40});
    chassis.waitUntilDone();

    // back into long goal while extaking
    chassis.moveToPoint(-35, -47, 1500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(-15, -47, 2000, {.forwards = false, .maxSpeed = 30});
    pto.set_value(true);
    chassis.waitUntilDone();
    
    // wing blocks into control
    pto.set_value(false);
    wing.set_value(true);
    chassis.moveToPoint(-35, -47, 750);
    chassis.chainToHeading(0, 750);
    chassis.chainToPoint(-35, -35, 750);
    chassis.chainToHeading(270, 750);
    chassis.chainToPoint(-10, -35, 1500, {.forwards = false});
    wing.set_value(false);
    while(true){
        chassis.moveToPoint(-4, -35, 200, {.forwards = false});
        chassis.waitUntilDone();
    }
}


/**
ose() * @brief runs the left side 4-3 split autonomous.
 *
 *
 *
 *
 *
 *
 */
void left_43Split(){
    // intake the 3-block stack
    chassis.setPose(-47, 17.81, 75);
    conveyorControl(600);
    chassis.chainToPoint(-14.86, 24.39, 2000, {.maxSpeed = 60});

    // align with the middle goal
    chassis.chainToPoint(-22.75, 22.75, 1000, {.forwards = false});
    chassis.turnToHeading(315, 750);
    chassis.moveToPoint(-5, 5, 800, {.forwards = false});
    chassis.waitUntilDone();

    // back into mid goal while extaking
    trapdoor.set_value(true);
    chassis.moveToPoint(-5, 5, 1500, {.forwards = false, .maxSpeed = 50});
    chassis.waitUntilDone();

    // matchload 3 blocks
    trapdoor.set_value(false);
    chassis.chainToPoint(-48, 48, 1500);
    chassis.chainToHeading(270, 750);
    scraper.set_value(true);
    chassis.moveToPoint(-999, 48, 1200, {.maxSpeed = 40});
    chassis.waitUntilDone();

    // align with long goal and score
    chassis.chainToPoint(-20, 48, 1500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(-15, 48, 1500, {.forwards = false, .maxSpeed = 30});
    pto.set_value(true);
    chassis.waitUntilDone();

    // wing blocks into control
    chassis.chainToPoint(-42, 48, 1000);
    pto.set_value(false);
    wing.set_value(true);
    chassis.chainToHeading(240, 750);
    chassis.chainToPoint(-25, 61, 1500, {.forwards = false});
    chassis.turnToHeading(270, 750);
    wing.set_value(false);
    while(true){
        chassis.moveToPoint(-4, 61, 200, {.forwards = false});
        chassis.waitUntilDone();
    }
}

/**
 * @brief runs the right side 4-3 split autonomous.
 *
 *
 *
 *
 *
 *
 */
void right_43Split(){
    chassis.setPose(-51.5, -10, 120);
    chassis.moveToPoint(-22, -23, 2000, {.maxSpeed = 70});
    conveyorControl(600);
    chassis.turnToHeading(230, 2000);
    chassis.moveToPoint(-5, -5, 2000, {.forwards = false});
    ptoChange();
    chassis.waitUntilDone();
    chassis.moveToPoint(-45, -47, 2000, {.forwards = false});
    chassis.moveToPoint(-62, -47, 2000);
    conveyorControl(600);
    chassis.moveToPoint(-29, -47, 2000, {.forwards = false});
    ptoChange();
    chassis.moveToPoint(-33, -59, 2000);
    chassis.turnToHeading(90, 2000);
    wing.set_value(true);
    chassis.moveToPoint(-8, -47, 2000, {.forwards = false});
    wing.set_value(false);
}

/**
 * @brief runs the solo AWP (left side).
 *
 *
 *
 *
 *
 *
 */
void awp(){
    // intake the matchloader
    chassis.setPose(-55, 16, 0);
    chassis.chainToPoint(-55, 47, 1000);
    chassis.chainToHeading(270, 750);
    scraper.set_value(true);
    conveyorControl(600);
    chassis.moveToPoint(-100, 47, 1500, {.maxSpeed = 40});
    chassis.chainToPoint(-34.5, 47, 1500, {.forwards = false});
    chassis.waitUntilDone();
    
    // move to align with long goal & score
    chassis.moveToPoint(-15, 47, 1500, {.forwards = false, .maxSpeed = 30});
    ptoChange();
    chassis.waitUntilDone();
    
    //intake the 3-block stack x2
    chassis.chainToPoint(-34.5, 47, 1500);
    chassis.turnToHeading(154, 750);
    chassis.chainToPoint(-22.5, 22, 1000);
    chassis.turnToHeading(180, 750);

    // score in mid-top goal
    chassis.chainToPoint(-22.5, 22, 1500);
    chassis.turnToHeading(227.5, 750);
    chassis.moveToPoint(0, 0, 1500, {.forwards = false});
    ptoChange();
    chassis.waitUntilDone();

    // intake the matchloader
    chassis.chainToPoint(-47, -47, 2000);
    chassis.turnToHeading(270, 750);
    chassis.moveToPoint(-100, -47, 1500, {.maxSpeed = 40});
    chassis.chainToPoint(-33, -47, 1500, {.forwards = false, .maxSpeed = 30});
    ptoChange();
    chassis.waitUntilDone();
}

/**
 * @brief runs the programming skills autonomous.
 *
 *
 *
 *
 *
 *
 */
void skills(){
       /* 96 Route attempt skills auton*/
   // in-take parking
   chassis.setPose(-63,13,0);
   chassis.chainToPoint(-63,25,1500);
   conveyorControl(600);
   pros::delay(750);
   scraper.set_value(true);
   pros::delay(750);
   scraper.set_value(false);
   // Pick up 1 block from the 4-stack
   chassis.chainToHeading(85, 750);
   chassis.chainToPoint(-27, 27, 1500);
   chassis.chainToHeading(315, 1500);
   chassis.moveToPoint(-12, 12, 750, {.forwards = false});
   chassis.waitUntilDone();
   // midtake here (dont have command)
   chassis.moveToPoint(0,0, 3000, {.forwards = false, .maxSpeed = 30});
   // Matchload all blocks
   chassis.chainToPoint(-47, -47, 1500);
   chassis.chainToHeading(270, 1500);
   chassis.waitUntilDone();
   scraper.set_value(true);
   chassis.moveToPoint(-60, -47, 1500);
   chassis.moveToPoint(-99, -47, 3000, {.maxSpeed = 40});
   // Extake on other side of long goal
   chassis.chainToHeading(62, 750);
   chassis.chainToPoint(-33, 60, 1500);
   chassis.chainToHeading(90, 750);
   chassis.chainToPoint(42, 60, 1500);
   chassis.chainToHeading(180, 750);
   chassis.chainToPoint(42, 47, 750);
   chassis.chainToHeading(90, 750);
   chassis.moveToPoint(29, 47, 1500, {.forwards = false});
   chassis.waitUntilDone();
   chassis.moveToPoint(15, 47, 3000, {.forwards = false, .maxSpeed = 30});
   pto.set_value(true);
   pros::delay(1500);
   pto.set_value(false);
   scraper.set_value(true);
   chassis.moveToPoint(58, 47, 750);
   chassis.waitUntilDone();
   chassis.moveToPoint(99, 47, 3000, {.maxSpeed = 40});
   chassis.moveToPoint(42, 47, 750, {.forwards = false});
   chassis.waitUntilDone();
   chassis.moveToPoint(29, 47, 750, {.forwards = false});
   pto.set_value(true);
   chassis.moveToPoint(15, 47, 3000, {.forwards = false, .maxSpeed = 30});
   pros::delay(1500);
   pto.set_value(false);
   scraper.set_value(false);
   // Clear alliance side parking
   chassis.moveToPoint(42, 47, 750);
   chassis.chainToHeading(132, 750);
   chassis.chainToPoint(66, 24, 1500);
   chassis.waitUntilDone();
   chassis.chainToPoint(66, -33, 1500);
   // Score Lower Center Goal and intake one block from 4-stack
   conveyorControl(600);
   chassis.chainToHeading(289, 750);
   chassis.chainToPoint(27, -19, 1500);
   chassis.chainToHeading(0, 750);
   conveyorControl(0);
   chassis.chainToPoint(27, 27, 750);
   chassis.chainToHeading(225, 750);
   chassis.moveToPoint(9,9,750);
   chassis.waitUntilDone();
   chassis.moveToPoint(0,0, 3000);
   conveyorControl(-600);
   // Matchload on long goal
   chassis.chainToPoint(27,27, 1500, {.forwards = false});
   chassis.chainToHeading(165, 750);
   chassis.chainToPoint(47, -47, 1500);
   chassis.waitUntilDone();
   chassis.chainToHeading(90, 750);
   chassis.moveToPoint(59, -47, 1500);
   scraper.set_value(true);
   conveyorControl(600);
   chassis.moveToPoint(99, -47, 3000, {.maxSpeed = 40});
   // Extake on other side of long goal
   chassis.moveToPoint(47, -47, 1500, {.forwards = false});
   chassis.chainToHeading(235, 750);
   chassis.chainToPoint(29, -60, 750);
   chassis.chainToHeading(270, 750);
   chassis.chainToPoint(-37, -60, 1500);
   chassis.chainToHeading(0, 750);
   chassis.moveToPoint(-37, -47, 1500);
   chassis.chainToHeading(270, 750);
   chassis.moveToPoint(-30, -47, 750, {.forwards = false});
   chassis.waitUntilDone();
   pto.set_value(true);
   chassis.moveToPoint(-15, -47, 3000, {.forwards = false, .maxSpeed = 30});
   // Matchload and extake on long goal
   chassis.moveToPoint(-37, -47, 750);
   
   
   

    /* skills auton - right start  -------------  maximum total time to take 
        //** left alliance side /
    chassis.setPose(-48, -14.2, 180);
    scraper.set_value(true);
    chassis.moveToPoint(-48, -48, 2000, {.maxSpeed = 80});                            // 1.5
    chassis.turnToHeading(270, 750);         
    intake.move_velocity(600);                       // 2.25
    chassis.moveToPoint(-120, -48, 2500, {.maxSpeed = 40});                            // 3.25
    pros::delay(2500);                                              // 6.25
    scraper.set_value(false);
    rabbit.set_value(true);
    chassis.setPose(-57, -48, 270);
    chassis.moveToPoint(-48, -48, 1000, {.forwards = false});       // 7.25
    chassis.turnToHeading(225, 750);                                // 9
    intake.brake();
    chassis.moveToPoint(-36, -57, 1000, {.maxSpeed = 80});                    // 10
    chassis.turnToHeading(89, 1000);                                 // 10.75
    chassis.waitUntilDone();
    chassis.setPose(-36, -62, 90);
    chassis.moveToPoint(46, -61, 2600, {.maxSpeed = 90});                         // 12.25
    chassis.turnToHeading(180, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(46, -999, 1000, {.maxSpeed = 30});
    chassis.setPose(44, -57.8, 180);
    chassis.moveToPoint(44, -48, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.turnToHeading(90, 800);
    chassis.waitUntilDone();
    chassis.moveToPoint(25, -48, 750, {.forwards = false, .maxSpeed = 60});
    intake.move_velocity(600);
    chassis.waitUntilDone();
    scraper.set_value(true);
    if(intake.get_actual_velocity() < 50){
        pros::delay(250);
        intake.move_velocity(-200);
        pros::delay(500);
    }                                              // 16.5
    intake.move_velocity(600);
    extakeT.move_velocity(600);
    chassis.moveToPoint(10, -48, 2500, {.forwards = false, .maxSpeed = 5});                                              // 18.5
    chassis.waitUntilDone();
    intake.brake();
    extakeT.brake();
        //** left opponent side /
    intake.move_velocity(600);
    chassis.moveToPoint(55, -48, 1000, {.maxSpeed = 80});
    chassis.moveToPoint(120, -48, 1500, {.maxSpeed = 40});                             // 19.5
    pros::delay(2500);                                              // 22.5
    chassis.moveToPoint(25, -48, 1500, {.forwards = false});        // 24
    chassis.waitUntilDone();
    if(intake.get_actual_velocity() < 50){
        pros::delay(250);
        intake.move_velocity(-200);
        pros::delay(500);
    }
    intake.move_velocity(600);
    extakeT.move_velocity(600);
    chassis.moveToPoint(10, -48, 2500, {.forwards = false, .maxSpeed = 5});
    chassis.waitUntilDone();
    extakeT.brake();
        //** right opponent side /
    chassis.moveToPoint(41, 48.5, 3000, {.maxSpeed = 80});
    chassis.turnToHeading(90, 750);                                 // 31.75
    chassis.waitUntilDone();
    chassis.setPose(-48, -48, 270);
    chassis.moveToPoint(-120, -48, 2500, {.maxSpeed = 40});
    pros::delay(2500);                                              // 6.25
    scraper.set_value(false);
    rabbit.set_value(true);
    chassis.setPose(-57, -47, 270);
    chassis.moveToPoint(-48, -48, 1000, {.forwards = false});       // 7.25
    chassis.turnToHeading(225, 750);                                // 9
    intake.brake();
    chassis.moveToPoint(-36, -59, 1000, {.maxSpeed = 80});                    // 10
    chassis.turnToHeading(90, 1000);                                 // 10.75
    chassis.waitUntilDone();
    chassis.setPose(-36, -62, 90);
    chassis.moveToPoint(46, -62, 2600, {.maxSpeed = 90});                         // 12.25
    chassis.turnToHeading(180, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(46, -999, 1000, {.maxSpeed = 30});
    chassis.setPose(44, -57.8, 180);
    chassis.moveToPoint(44, -48, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.turnToHeading(90, 800);
    chassis.waitUntilDone();
    chassis.moveToPoint(25, -48, 750, {.forwards = false, .maxSpeed = 60});
    intake.move_velocity(600);
    chassis.waitUntilDone();
    scraper.set_value(true);
    if(intake.get_actual_velocity() < 50){
        pros::delay(250);
        intake.move_velocity(-200);
        pros::delay(500);
    }         
    intake.move_velocity(600);                                     // 16.5
    extakeT.move_velocity(600);
    chassis.moveToPoint(10, -48, 2500, {.forwards = false, .maxSpeed = 15});                           
    chassis.waitUntilDone();
    extakeT.brake();
        //** right opponent side /
    chassis.moveToPoint(55, -48, 1000, {.maxSpeed = 80});
    chassis.moveToPoint(120, -48, 1500, {.maxSpeed = 40});                             // 19.5
    pros::delay(2500);                                              // 22.5
    chassis.moveToPoint(25, -48, 1500, {.forwards = false});        // 24
    chassis.waitUntilDone();
    if(intake.get_actual_velocity() < 50){
        pros::delay(250);
        intake.move_velocity(-200);
        pros::delay(250);
    }
    pros::delay(750);
    intake.move_velocity(600);
    extakeT.move_velocity(600);
    scraper.set_value(false);
    chassis.moveToPoint(10, -48, 2500, {.forwards = false, .maxSpeed = 20});
    chassis.waitUntilDone();
    intake.brake();
    extakeT.brake();
    chassis.setPose(-25, 48, 270);
        //** parking /
    chassis.setPose(-25, 48, 270);
    chassis.moveToPoint(-48, 44, 1500, {.maxSpeed = 80});
    chassis.turnToHeading(235, 750);
    chassis.moveToPose(-60, 18.5, 180, 1500, {.maxSpeed = 80});
    odomLift.set_value(true);
    intake.move_velocity(600);
    chassis.waitUntilDone();
    while(chassis.getPose().y >= -10){
        scraper.set_value(true);
        chassis.moveToPoint(-63, -20, 200);
        chassis.waitUntilDone();
    }
    scraper.set_value(false);
    extakeT.move_velocity(600);
    chassis.moveToPoint(-63, -9, 1000, {.forwards = false});
    
    
    /*
    
    chassis.moveToPoint(62, 48, 1500);                              // 33.25
    intake.move_velocity(600);
    pros::delay(3000);                                              // 36.25
    scraper.set_value(false);
    chassis.moveToPoint(48, 48, 1000, {.forwards = false});         // 37.25
    chassis.turnToHeading(45, 750);                                 // 38
    chassis.moveToPoint(36.095, 59.905, 1000);                      // 41
    chassis.turnToHeading(270, 750);                                // 41.75
    chassis.moveToPoint(-50, 59.905, 1500);                         // 43.25
    chassis.turnToHeading(45, 750);                                 // 44
    chassis.moveToPose(-25, 48, 270, 1500);                         // 45.5
    pros::delay(2000);                                              // 47.5
    extakeT.move_velocity(600); 
    pros::delay(2000);                                              // 49.5
    extakeT.brake();
        //** right alliance side /
    scraper.set_value(true);
    chassis.moveToPoint(-62, 48, 1000);                             // 50.5
    pros::delay(3000);                                              // 53.5
    scraper.set_value(false);
    chassis.moveToPoint(-25, 48, 1500, {.forwards = false});        // 55
    pros::delay(2000);                                              // 57
    extakeT.move_velocity(600);
    pros::delay(2000);                                              // 59
    intake.brake();
    extakeT.brake();
        //** clear and park /
    chassis.moveToPoint(-36, 48, 1000);                             // 60
    chassis.turnToHeading(180, 500);                                // 60.5
    chassis.moveToPoint(-36, 0, 1500);                              // 62
    chassis.turnToHeading(270, 500);                                // 62.5
    scraper.set_value(true);
    chassis.moveToPoint(-62, 0, 10000);                             // until end
    intake.move_velocity(600);
    extakeT.move_velocity(600);                                     // end
        //** ideally 79 points /
    // */
}
