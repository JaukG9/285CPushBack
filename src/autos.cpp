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

}

/**
 * @brief runs the right side 4 block rush autonomous.
 *
 * Intakes the group of 3 blocks that start on right side of the field and outtakes
 * them, with the preload, into the long goal for a total of 4 blocks in the right
 * long goal. Then, wings the blocks into control.
 */
void right_4Rush(){
    chassis.setPose(-50.715, 4.034, 0);
    conveyorControl(600);
    
    
    
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
   /* left 7 Rush block ---------------------------------- ~16 seconds (Need to fix directions)*/
    chassis.setPose(-140, 20, 65);
    conveyorControl(600);
    chassis.chainToHeading(320, 750);
    chassis.chainToPoint(-58, 55, 1500); // 1.5 
    conveyorControl(0);
    chassis.chainToHeading(270, 750);
    chassis.chainToPoint(-120,120, 1500); // 1.5
    conveyorControl(600);
    chassis.chainToPoint(-235, 120, 1500, {.maxSpeed = 50}); // 1.5
    pros::delay(250); // .25
    conveyorControl(0);
    chassis.chainToPoint(-67, 120, 1500, {.forwards = false}); // 1.5
    chassis.waitUntilDone();
    chassis.chainToPoint(-10, 120, 1500, {.forwards = false, .maxSpeed = 10}); // 1.5
    pto.set_value(true);
    pros::delay(1500); // 2
    pto.set_value(false);
    chassis.chainToPoint(-90, 120, 1500); // 1.5
    chassis.chainToHeading(0, 750);
    chassis.chainToPoint(-90, 150, 1500); // 1.5
    chassis.chainToHeading(270, 750);
    wing.set_value(true);
    chassis.chainToPoint(-30, 150, 1500, {.forwards = false}); // 1.5
    pros::delay(1500); // 1.5
    wing.set_value(false);
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
    /* Right 7 Rush block ---------------------------------- ~*/
    chassis.setPose(-54,-7,122);
    conveyorControl(600);
    chassis.chainToPoint(-24, 24, 1500);
    pros::delay(250);
    conveyorControl(0);
    chassis.chainToHeading(224, 750);
    chassis.chainToPoint(-48, -47, 1500);
    chassis.waitUntilDone();
    chassis.chainToHeading(270, 750);
    /* for myself Paused at (-48, -47, 1500) before the matchloader + long score*/
    scraper.set_value(true);
    chassis.chainToPoint(-93, -47, 1500, {.maxSpeed = 50});
    chassis.chainToPoint(-30, -47, 1500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.chainToPoint(0, -47, 1500, {.forwards = false, .maxSpeed = 10});
    pto.set_value(true);
    pros::delay(1500);
    pto.set_value(false);
    chassis.chainToPoint(-30, -47, 1500);
    chassis.chainToHeading(0, 750);
    chassis.chainToPoint(-30, -35, 750);
    chassis.chainToHeading(90, 750);
    wing.set_value(true);
    chassis.chainToPoint(-5, -35, 750);
    pros::delay(1500);
    wing.set_value(false);
}

/**
 * @brief runs the left side 4-3 split autonomous.
 *
 *
 *
 *
 *
 *
 */
void left_43Split(){
    chassis.setPose(-47, 17.81, 75);
    conveyorControl(600);
    chassis.chainToPoint(-14.86, 24.39, 2000, {.maxSpeed = 60});
    

    /*
    chassis.setPose(-47, 17.81, 75);
    chassis.moveToPoint(-14.86, 24.39, 2000, {.maxSpeed = 52});                       // 1
    conveyorControl(600);
    pros::delay(800);                                               // 1.8
    scraper.set_value(true);
    chassis.moveToPose(-24, 24, 315, 1500, {.forwards = false});    // 3.3
    chassis.moveToPoint(-5, 5, 2400, {.forwards = false, .maxSpeed = 50});        // 5
    pros::delay(800);                                              // 6
    trapdoor.set_value(true);
    pto.set_value(true);
    pros::delay(1200);                                              // 7
    conveyorControl(0);
    pto.set_value(false);
    trapdoor.set_value(false);
    chassis.moveToPoint(-44, 49, 1500, {.maxSpeed = 80});                             // 9
    chassis.turnToHeading(270, 500);
    conveyorControl(600);
    chassis.moveToPoint(-62, 48.25, 1200, {.maxSpeed = 40});
    chassis.waitUntilDone();                                     // 12
    chassis.moveToPoint(-15, 48.5, 1500, {.forwards = false});        // 13.5
    pros::delay(750);
    pto.set_value(true);
    scraper.set_value(false);
    pros::delay(1000);
    chassis.moveToPoint(-42, 48.5, 1000);
    wing.set_value(true);
    chassis.turnToHeading(240, 750);
    chassis.moveToPoint(-25, 61.5, 2000, {.forwards = false});
    chassis.turnToHeading(270, 750);
    chassis.moveToPoint(-4, 61.5, 10000, {.forwards = false});
    wing.set_value(false);
    */
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
