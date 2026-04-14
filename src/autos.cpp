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
    //in-take the 3 block stack
    chassis.setPose(-47, 17, 90);
    conveyorControl(600);
    chassis.chainToPoint(-14.86, 22.2, 1500);
    pros::delay(500);
    scraper.set_value(true);
    wing.set_value(true);

    //move to align with the long goal
    chassis.chainToHeading(327, 750);
    chassis.chainToPoint(-41, 48, 1500);
    scraper.set_value(false);
    chassis.chainToHeading(270, 750);
    chassis.chainToPoint(-30, 48, 1000, {.forwards = false});
    
    //back into long goal while extaking
    chassis.moveToPoint(-15, 48, 1500, {.forwards = false, .maxSpeed = 30});
    pto.set_value(true);
    chassis.waitUntilDone();

    // wing blocks into control
    pto.set_value(false);
    conveyorBrake();
    scraper.set_value(false);
    chassis.moveToPoint(-42, 48, 750);
    chassis.chainToPose(-35, 56, 270, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.chainToPoint(-12, 56, 1500, {.forwards = false, .maxSpeed = 80});
    wing.set_value(false);
    while(true){
        chassis.moveToPoint(-12, 55, 200, {.forwards = false});
        chassis.waitUntilDone();
    }
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
    chassis.setPose(-47, -18, 90);
    conveyorControl(600);
    chassis.chainToPoint(-14.86, -23.39, 1500);
    pros::delay(500);
    scraper.set_value(true);
    wing.set_value(true);

    // move to align with long goal
    chassis.chainToHeading(235, 750);
    chassis.chainToPoint(-40, -48, 1500);
    scraper.set_value(false);
    chassis.chainToHeading(270, 750);
    chassis.chainToPoint(-30, -48, 1200, {.forwards = false});

    // back into long goal while extaking
    chassis.moveToPoint(-15, -48, 1500, {.forwards = false, .maxSpeed = 30});
    pto.set_value(true);
    chassis.waitUntilDone();

    // wing blocks into control
    pto.set_value(false);
    scraper.set_value(false);
    chassis.moveToPoint(-42, -48, 750);
    chassis.chainToPose(-35, -40, 270, 1500, {.forwards = false});
    chassis.chainToPoint(-12, -40, 1500, {.forwards = false, .maxSpeed = 80});
    wing.set_value(false);
    while(true){
        chassis.moveToPoint(-12, -41, 200, {.forwards = false});
        chassis.waitUntilDone();
    }
}

/**
 * @brief runs the left side 7 block rush autonomous.
 *
 * Intakes the group of 3-block stacks that start on the left side of the field and intakes
 * another 3 blocks from the matchloader and extakes all 7 into the long goal including preload.
 * Then, wings the blocks into control.
 *
 */
void left_7Rush(){
    //in-take the 3 block stack
    chassis.setPose(-47, 16.5, 90);
    conveyorControl(600);
    chassis.chainToPoint(-14.86, 21.5, 1500);
    pros::delay(500);
    scraper.set_value(true);
    wing.set_value(true);

    //move to align with the long goal
    chassis.chainToHeading(315, 750);
    chassis.chainToPoint(-48, 48, 1500);
    chassis.chainToHeading(270, 750);
    scraper.set_value(true);
    chassis.moveToPoint(-100, 47, 1500, {.maxSpeed = 40});

    // back into long goal while extaking
    chassis.chainToPoint(-35, 47, 1500, {.forwards = false});
    chassis.moveToPoint(-15, 47, 2000, {.forwards = false, .maxSpeed = 30});
    pto.set_value(true);
    chassis.waitUntilDone();

    // wing blocks into control
    pto.set_value(false);
    scraper.set_value(false);
    chassis.moveToPoint(-42, 48, 750);
    chassis.chainToPose(-35, 56, 270, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.chainToPoint(-20, 56, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.chainToPoint(-12, 56, 1500, {.forwards = false, .maxSpeed = 50});
    wing.set_value(false);
    while(true){
        chassis.moveToPoint(-12, 55, 200, {.forwards = false});
        chassis.waitUntilDone();
    }
}

/**
 * @brief Runs the right side 7-block rush autonomous.
 *
 * Intakes a group of 3-block stacks on the right side of the field, along with
 * 3 blocks from the match loader, and extakes all 7 into the long goal including the preload.
 * Then, wings the blocks into the control zone.
 *
 */
void right_7Rush(){
    // intake the 3-block stack
    chassis.setPose(-47, -18, 90);
    conveyorControl(600);
    chassis.chainToPoint(-14.86, -23, 1500);
    pros::delay(600);
    scraper.set_value(true);
    wing.set_value(true);

    // move to align with long goal intaking only 3 blocks
    chassis.chainToHeading(235, 750);
    chassis.chainToPoint(-48, -48, 1500);
    chassis.chainToHeading(270, 750);
    chassis.moveToPoint(-99, -48, 900, {.maxSpeed = 40});
    chassis.waitUntilDone();

    // back into long goal while extaking
    chassis.moveToPoint(-30, -47, 1500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(-15, -47, 2000, {.forwards = false, .maxSpeed = 30});
    pto.set_value(true);
    chassis.waitUntilDone();
    conveyorBrake();
    
    // wing blocks into control
    pto.set_value(false);
    scraper.set_value(false);
    chassis.moveToPoint(-42, -48, 750);
    chassis.chainToPose(-35, -39, 270, 1500, {.forwards = false});
    wing.set_value(false);
    chassis.chainToPoint(-12, -39, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.chainToPoint(-12, -40, 1500, {.forwards = false, .maxSpeed = 40});
    while(true){
        chassis.moveToPoint(-12, -41, 200, {.forwards = false});
        chassis.waitUntilDone();
    }
}

/**
 * @brief runs the left side 4-3 split autonomous.
 *
 * Intakes the 3-block stack on the left side of the field and extakes them into
 * the mid goal. Then match loads 3 blocks from the match loader and extakes them
 * into the long goal. Finally, wings the remaining blocks into the control zone.
 *
 *
 */
void left_43Split(){
    //in-take the 3 block stack
    chassis.setPose(-47, 18, 90);
    conveyorControl(600);
    chassis.chainToPoint(-14.86, 22.2, 1500);
    pros::delay(500);
    scraper.set_value(true);
    wing.set_value(true);

    // align with the middle goal
    chassis.chainToPose(-15, 15, 315, 1250, {.forwards = false});
    chassis.moveToPoint(-5, 5, 670, {.forwards = false, .maxSpeed = 80});
    pros::delay(200);
    trapdoor.set_value(true);
    chassis.waitUntilDone();

    // back into mid goal while extaking
    chassis.moveToPoint(-5, 5, 2000, {.forwards = false, .maxSpeed = 50});
    chassis.waitUntilDone();

    // matchload 3 blocks
    trapdoor.set_value(false);
    chassis.chainToPoint(-48, 48.5, 1500);
    chassis.chainToHeading(270, 750);
    scraper.set_value(true);
    chassis.moveToPoint(-99, 48.5, 1000, {.maxSpeed = 40});
    chassis.waitUntilDone();

    // back into long goal while extaking
    chassis.moveToPoint(-30, 48.5, 1500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(-15, 48.5, 2000, {.forwards = false, .maxSpeed = 30});
    pto.set_value(true);
    chassis.waitUntilDone();
    conveyorBrake();

    // wing blocks into control
    pto.set_value(false);
    scraper.set_value(false);
    chassis.moveToPoint(-42, 48, 750);
    chassis.chainToPose(-35, 56, 270, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.chainToPoint(-12, 56, 1500, {.forwards = false, .maxSpeed = 80});
    wing.set_value(false);
    while(true){
        chassis.moveToPoint(-11, 55, 200, {.forwards = false});
        chassis.waitUntilDone();
    }
}

/**
 * @brief runs the right side 4-3 split autonomous.
 *
 * Intakes the 3-block stack on the right side of the field and scores them into
 * the low goal. Then match loads 3 blocks from the match loader and extakes them
 * into the long goal. Finally, wings the remaining blocks into the control zone.
 *
 *
 */
void right_43Split(){
    // intake the 3-block stack
    chassis.setPose(-47.5, -16.5, 90);
    conveyorControl(600);
    chassis.chainToPoint(-14.86, -23, 1500);
    pros::delay(600);
    scraper.set_value(true);
    wing.set_value(true);

    // align with the low goal
    chassis.chainToPose(-28, -28, 45, 1250, {.forwards = false});
    pros::delay(350);
    scraper.set_value(false);
    chassis.chainToPoint(-5, -5, 1250, {.maxSpeed = 40});
    chassis.waitUntilDone();
    conveyorControl(-400);
    intakeFunnel.set_value(true);
    pros::delay(1500);

    // matchload 3 blocks
    intakeFunnel.set_value(false);
    conveyorControl(600);
    chassis.moveToPoint(-42, -48, 1500, {.forwards = false});
    pros::delay(400);
    scraper.set_value(true);
    chassis.turnToHeading(270, 750);
    chassis.moveToPoint(-99, -48, 1500, {.maxSpeed = 40});
    chassis.waitUntilDone();

    // align with long goal and score
    chassis.chainToPoint(-33, -48, 1500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(-15, -48, 1500, {.forwards = false, .maxSpeed = 30});
    pto.set_value(true);
    chassis.waitUntilDone();

    // wing blocks into control
    pto.set_value(false);
    scraper.set_value(false);
    chassis.moveToPoint(-42, -48, 750);
    chassis.chainToPose(-35, -39, 270, 1500, {.forwards = false});
    wing.set_value(false);
    chassis.chainToPoint(-12, -39, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.chainToPoint(-12, -40, 1500, {.forwards = false, .maxSpeed = 40});
    while(true){
        chassis.moveToPoint(-12, -41, 200, {.forwards = false});
        chassis.waitUntilDone();
    }
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
void left_awp(){
    // intake the matchloader
    chassis.setPose(-55, 16, 0);
    chassis.chainToPoint(-55, 48, 1000);
    chassis.chainToHeading(270, 750);
    scraper.set_value(true);
    conveyorControl(600);
    chassis.moveToPoint(-100, 48, 1500, {.maxSpeed = 40});
    chassis.chainToPoint(-34.5, 48, 1500, {.forwards = false});
    chassis.waitUntilDone();
    
    // move to align with long goal & score
    chassis.moveToPoint(-15, 48, 1500, {.forwards = false, .maxSpeed = 30});
    pto.set_value(true);
    pros::delay(1000);
    pto.set_value(false);
    
    //intake the 3-block stack x2
    chassis.chainToPoint(-34.5, 48, 1500);
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
    chassis.chainToPoint(-47, -48, 2000);
    chassis.turnToHeading(270, 750);
    chassis.moveToPoint(-100, -48, 1500, {.maxSpeed = 40});
    chassis.chainToPoint(-33, -48, 1500, {.forwards = false, .maxSpeed = 30});
    pto.set_value(true);
    pros::delay(1000);
    pto.set_value(false);

    // wing blocks into control
    pto.set_value(false);
    scraper.set_value(false);
    chassis.moveToPoint(-42, 48, 750);
    chassis.chainToPose(-35, 56, 270, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.chainToPoint(-12, 56, 1500, {.forwards = false, .maxSpeed = 80});
    wing.set_value(false);
    while(true){
        chassis.moveToPoint(-12, 55, 200, {.forwards = false});
        chassis.waitUntilDone();
    }
}

/**
 * @brief runs the solo AWP (right side).
 *
 * Intakes three blocks from the match loader and extakes them into the long goal along with
 * the preload. Then picks up three blocks from the three stacks twice and extakes
 * them into the mid goal. Finally, intakes three blocks from the match loader and extakes
 * six blocks into the long goal.
 *
 *
 */

void right_awp(){
    // intake matchloader
    chassis.setPose(-55, -8, 180);
    conveyorControl(600);
    chassis.chainToPoint(-55, -48, 1500);
    chassis.turnToHeading(270, 750);
    scraper.set_value(true);
    chassis.moveToPoint(-99, -48, 1500, {.maxSpeed = 40});
    chassis.waitUntilDone();

    // align with long-goal and score
    chassis.moveToPoint(-30, -48, 1500, {.forwards = false});
    scraper.set_value(false);
    chassis.moveToPoint(-15, -48, 4000, {.forwards = false, .maxSpeed = 30});
    pto.set_value(true);
    chassis.waitUntilDone();
    pto.set_value(false);

    // Pick up 3-stack twice
    chassis.moveToPoint(-55, -48, 1500);
    chassis.chainToHeading(50, 750);
    chassis.chainToPoint(-22, -22, 1500);
    chassis.chainToHeading(0, 750);
    chassis.moveToPoint(-22, 22, 1500);
    
    // Score in mid-goal (3 blocks only)
    chassis.turnToHeading(315, 1500);
    chassis.moveToPoint(-12, 12, 1500, {.forwards = false});
    trapdoor.set_value(true);
    conveyorControl(300);
    chassis.moveToPoint(0,0, 1500, {.forwards = false, .maxSpeed = 30});
    pto.set_value(true);
    chassis.waitUntilDone();

    // intake matchloader (3 blocks)
    conveyorControl(600);
    chassis.chainToPoint(-45, 48, 1500);
    chassis.turnToHeading(270, 750);
    scraper.set_value(true);
    chassis.moveToPoint(-60, 48, 1500);
    chassis.moveToPoint(-99, 48, 1500, {.maxSpeed = 40});
    chassis.waitUntilDone();
    
    // score in long-goal (6 blocks)
    chassis.moveToPoint(-30, 48, 1500, {.forwards = false});
    chassis.moveToPoint(-15, 48, 15000, {.forwards = false, .maxSpeed = 30});
    pto.set_value(true);
    chassis.waitUntilDone();
}


/**
 * @brief runs the programming skills autonomous.
 *
 * Parks in the alliance parking zone and intakes the parking blocks, then picks up
 * and scores a block from the 4-stack into the mid goal. Intakes the 4-stack on the
 * opposite side and extakes into the long goal, then match loads and extakes blocks
 * across both long goals twice. Finally, scores in the mid goal, repeats the match load
 * and extake cycle, and parks in the alliance parking zone.
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


int autonomousSelection = -1; // auton selection var

/**
 * @brief displays auton selector so user can select autos from one program
 *
 * Uses custom Button object to display 4 buttons to the screen where the user can
 * choose which auton they use. The first screen displays options of (1) Left,
 * (2) Right, (3) Skip, and (4) Skills. Options (3) and (4) choose their respective
 * autonomous routines. Options (1) and (2) display 4 more options which are as
 * follows (a) 4 Rush, (b) 7 Rush, (c) Split, (d) AWP. In combination with the first
 * picked option, this chooses the respective autonomous routine.
 */
void autonSelector(void*){
    int autonomousType = 0;
    autonomousSelection = -1;
    bool autonTypeSelected = false, autonSelected = false;

    Button autonType[] = {
        Button(10, 10, 225, 105, "Left", pros::Color::pale_violet_red, pros::Color::black),
        Button(245, 10, 225, 105, "Right", pros::Color::cyan, pros::Color::black),
        Button(10, 125, 225, 105, "Skip", pros::Color::white, pros::Color::black),
        Button(245, 125, 225, 105, "Skills", pros::Color::light_green, pros::Color::black)
    };

    pros::screen::set_pen(pros::Color::light_gray);
    pros::screen::fill_rect(0, 0, 480, 240);
    while(!autonTypeSelected){
        for(int i = 0; i < 4; i++){
            autonType[i].render();
            if(autonType[i].isClicked()){
                autonomousType = i;
                autonTypeSelected = true;
            }
        }
        pros::delay(20);
    }
    pros::screen::erase();

    Button* auton = nullptr;
    static Button leftAutons[] = {
        Button(10, 10, 225, 105, "Left 4 Rush", pros::Color::peach_puff, pros::Color::black),
        Button(245, 10, 225, 105, "Left 7 Rush", pros::Color::peach_puff, pros::Color::black),
        Button(10, 125, 225, 105, "Left Split", pros::Color::peach_puff, pros::Color::black),
        Button(245, 125, 225, 105, "Left AWP", pros::Color::peach_puff, pros::Color::black)
    };
    static Button rightAutons[] = {
        Button(10, 10, 225, 105, "Right 4 Rush", pros::Color::peach_puff, pros::Color::black),
        Button(245, 10, 225, 105, "Right 7 Rush", pros::Color::peach_puff, pros::Color::black),
        Button(10, 125, 225, 105, "Right Split", pros::Color::peach_puff, pros::Color::black),
        Button(245, 125, 225, 105, "Right AWP", pros::Color::peach_puff, pros::Color::black)
    };

    switch(autonomousType){
        case 0: auton = leftAutons; break;
        case 1: auton = rightAutons; break;
        case 2: autonomousSelection = 9; autonSelected = true; break;
        case 3: autonomousSelection = 0; autonSelected = true; break;
    }

    pros::screen::set_pen(pros::Color::dark_gray);
    while(!autonSelected && autonTypeSelected){
        for(int i = 0; i < 4; i++){
            if(auton[i].isClicked()){
                autonomousSelection = i * 2 + 1 + autonomousType;
                auton[i].buttonColor = pros::Color::medium_violet_red;
                autonSelected = true;
            }
            auton[i].render();
        }

        pros::delay(20);
    }
}