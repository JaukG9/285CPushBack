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
    chassis.setPose(-47,17,75);
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
 * @brief runs the left side 4-3 split autonomous.
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
    pto.set_value(true);
    chassis.waitUntilDone();
    chassis.moveToPoint(-45, -47, 2000, {.forwards = false});
    chassis.moveToPoint(-62, -47, 2000);
    conveyorControl(600);
    chassis.moveToPoint(-29, -47, 2000, {.forwards = false});
    pto.set_value(false);
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
void awp_left(){
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
    pto.set_value(true);
    pros::delay(1000);
    pto.set_value(false);
    
    //intake the 3-block stack x2
    chassis.chainToPoint(-34.5, 47, 1500);
    chassis.turnToHeading(154, 750);
    chassis.chainToPoint(-22.5, 22, 1000);
    chassis.turnToHeading(180, 750);

    // score in mid-top goal
    chassis.chainToPoint(-22.5, 22, 1500);
    chassis.turnToHeading(227.5, 750);
    chassis.moveToPoint(0, 0, 1500, {.forwards = false});
    pto.set_value(true);
    pros::delay(1000);
    pto.set_value(false);

    // intake the matchloader
    chassis.chainToPoint(-47, -47, 2000);
    chassis.turnToHeading(270, 750);
    chassis.moveToPoint(-100, -47, 1500, {.maxSpeed = 40});
    chassis.chainToPoint(-33, -47, 1500, {.forwards = false, .maxSpeed = 30});
    pto.set_value(true);
    pros::delay(1000);
    pto.set_value(false);
}

/**
 * @brief runs the solo AWP (right side).
 *
 *
 *
 *
 *
 *
 */

void awp_right(){
    // intake the matchloader
    chassis.setPose(-55, -15, 180);
    chassis.chainToPoint(-55, -47, 1500);
    chassis.turnToHeading(270, 750);
    scraper.set_value(true);
    conveyorControl(600);
    chassis.moveToPoint(-100, -47, 300, {.maxSpeed = 40});
    
    // move to align with long goal & score
    chassis.chainToPoint(-33, -47, 1500, {.forwards = false});

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
/* 117 Route attempt skills auton*/
    // intake parking
    chassis.setPose(-45, 0, 270);
    odomLift.set_value(true);
    wing.set_value(true);
    chassis.moveToPoint(-99, 0, 2000, {.maxSpeed = 60});
    conveyorControl(600);
    chassis.waitUntilDone();
    chassis.chainToPoint(-45, 0, 1500, {.forwards = false});

    // Pick up 1 block from the 4-stack
    chassis.turnToHeading(138, 750);
    odomLift.set_value(false);
    chassis.setPose(-45, 0, 138);
    chassis.chainToPoint(-24, -24, 1500, {.maxSpeed = 80});
    chassis.chainToHeading(45, 1500);
    chassis.moveToPoint(-12, -12, 750);
    chassis.waitUntilDone();
    intakeFunnel.set_value(true);
    pto.set_value(true);
    conveyorControl(-200);
    chassis.moveToPoint(-1, -1, 3000, {.maxSpeed = 30});
    chassis.waitUntilDone();
    pto.set_value(false);
    conveyorBrake();

    // Pick up 4 blocks from the 4-stack on other side and score
    chassis.chainToPoint(-24, -24, 1500, {.forwards = false});
    chassis.chainToHeading(0, 750);
    chassis.chainToPoint(-24, 24, 1500);
    chassis.chainToHeading(330, 750);
    chassis.chainToPoint(-36, 48, 1000);
    chassis.chainToHeading(270, 750);
    chassis.moveToPoint(-25, 48, 750, {.forwards = false});
    pto.set_value(true);
    chassis.moveToPoint(-5, 48, 1500, {.forwards = false, .maxSpeed = 10});
    chassis.waitUntilDone();
    pto.set_value(false);

    // Matchload all blocks
    scraper.set_value(true);
    chassis.chainToPoint(-54, 48, 1000);
    chassis.moveToPoint(-99, 48, 3000, {.maxSpeed = 40});

    // Extake on other side of long goal
    chassis.moveToPoint(-48, 48, 1500, {.forwards = false});
    scraper.set_value(false);
    chassis.chainToHeading(30, 750);
    chassis.chainToPoint(-42, 60, 1500);
    chassis.chainToHeading(90, 750);
    chassis.chainToPoint(36, 60, 1500);
    chassis.chainToHeading(150, 750);
    chassis.chainToPoint(42, 48, 750);
    chassis.chainToHeading(90, 750);
    chassis.moveToPoint(25, 48, 750, {.forwards = false});
    chassis.waitUntilDone();
    pto.set_value(true);
    chassis.moveToPoint(5, 48, 1500, {.forwards = false, .maxSpeed = 10});
    chassis.waitUntilDone();
    pto.set_value(false);
    
    // Matchload all blocks
    scraper.set_value(true);
    chassis.moveToPoint(54, 48, 1000);
    chassis.moveToPoint(99, 48, 3000, {.maxSpeed = 40});
    chassis.moveToPoint(25, 48, 1000, {.forwards = false});
    chassis.waitUntilDone();
    pto.set_value(true);
    scraper.set_value(false);
    chassis.moveToPoint(5, 48, 1500, {.forwards = false, .maxSpeed = 10});
    chassis.waitUntilDone();
    pto.set_value(false);
    chassis.setPose(30, 48, 90);

    // Clear alliance side parking
    chassis.moveToPoint(42, 48, 750);
    chassis.chainToHeading(180, 750);
    chassis.chainToPoint(42, 0, 1500);
    chassis.turnToHeading(90, 750);
    odomLift.set_value(true);
    chassis.moveToPoint(99, 0, 2000, {.maxSpeed = 60});
    conveyorControl(600);
    chassis.chainToPoint(45, 0, 1500, {.forwards = false});

    // Score mid goal
    chassis.chainToHeading(215, 750);
    chassis.waitUntilDone();
    odomLift.set_value(false);
    chassis.setPose(45, 0, 215);
    chassis.chainToPoint(24, -24, 1500);
    chassis.chainToHeading(135, 750);
    chassis.moveToPoint(12, -12, 1500, {.forwards = false});
    chassis.waitUntilDone();
    conveyorControl(300);
    trapdoor.set_value(true);
    pto.set_value(true);
    chassis.moveToPoint(8, -8, 1500, {.forwards = false, .maxSpeed = 40});

    // Matchload on long goal
    chassis.moveToPoint(36, -48, 1500);
    chassis.turnToHeading(90, 750);
    chassis.setPose(-36, 48, 270);
    scraper.set_value(true);
    chassis.chainToPoint(-54, 48, 1000);
    chassis.moveToPoint(-99, 48, 3000, {.maxSpeed = 40});

    // Extake on other side of long goal
    chassis.moveToPoint(-48, 48, 1500, {.forwards = false});
    scraper.set_value(false);
    chassis.chainToHeading(30, 750);
    chassis.chainToPoint(-42, 60, 1500);
    chassis.chainToHeading(90, 750);
    chassis.chainToPoint(36, 60, 1500);
    chassis.chainToHeading(150, 750);
    chassis.chainToPoint(42, 48, 750);
    chassis.chainToHeading(90, 750);
    chassis.moveToPoint(25, 48, 750, {.forwards = false});
    chassis.waitUntilDone();
    pto.set_value(true);
    chassis.moveToPoint(5, 48, 1500, {.forwards = false, .maxSpeed = 10});
    chassis.waitUntilDone();
    pto.set_value(false);
    
    // Matchload all blocks
    scraper.set_value(true);
    chassis.moveToPoint(54, 48, 1000);
    chassis.moveToPoint(99, 48, 3000, {.maxSpeed = 40});
    chassis.moveToPoint(25, 48, 1000, {.forwards = false});
    chassis.waitUntilDone();
    pto.set_value(true);
    scraper.set_value(false);
    chassis.moveToPoint(5, 48, 1500, {.forwards = false, .maxSpeed = 10});
    chassis.waitUntilDone();
    pto.set_value(false);
    chassis.setPose(30, 48, 90);

    // park
    chassis.moveToPoint(42, 48, 750);
    chassis.chainToHeading(180, 750);
    chassis.chainToPoint(42, 0, 1500);
    chassis.turnToHeading(90, 750);
    odomLift.set_value(true);
    chassis.moveToPoint(99, 0, 2000, {.maxSpeed = 60});
    conveyorControl(600);
    chassis.waitUntilDone();
    chassis.setPose(60, 0, 90);
}
