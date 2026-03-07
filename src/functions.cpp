#include "main.h"
#include "lemlib/api.hpp"
#include "robot/drivetrain.h"
#include "robot/robot-config.h"
#include "robot/odometry.h"
#include "robot/functions.h"

/**
 * @brief runs the motors attached to the conveyor.
 *
 * This function runs the motors attached to the conveyor with a specified rpm,
 * running the robot's intake, conveyor, and (if PTO is activated) outtake.
 *
 * @param rpm the rotations per minute at which the motors should rotate.
 */
void conveyorControl(int rpm){
    conveyor1.move_velocity(rpm);
    conveyor2.move_velocity(rpm);
}

/**
 * @brief switches the state of the scraper piston.
 *
 * This function first changes the global boolean variable scraperActivated to 
 * its opposite state (true or false), and then sets the piston attached to the
 * scraper (on TriPort H) to the new value of scraperActivated (true = extended, 
 * false = unextended).
 *
 * This will extend or unextend the scraper.
 */
void scraperChange(){
    scraperActivated = !scraperActivated;
    scraper.set_value(scraperActivated);
}

/**
 * @brief switches the state of the trapdoor piston.
 *
 * This function first changes the global boolean variable trapdoorActivated to
 * its opposite state (true or false), and then sets the piston attached to the
 * trapdoor (on TriPort G) to the new value of trapdoorActivated (true = extended,
 * false = unextended).
 *
 * This will open or close the trapdoor.
 */
void trapdoorChange(){
    trapdoorActivated = !trapdoorActivated;
    trapdoor.set_value(trapdoorActivated);
}

/**
 * @brief switches the state of the wing piston.
 *
 * This function first changes the global boolean variable wingActivated to its
 * opposite state (true or false), and then sets the piston attached to the wing
 * (on TriPort F) to the new value of wingActivated (true = extended, false = unextended).
 *
 * This will extend or unextend the wing.
 */
void wingChange(){
    wingActivated = !wingActivated;
    wing.set_value(wingActivated);
}

/**
 * @brief switches the state of the PTO piston.
 *
 * This function first changes the global boolean variable ptoActivated to its
 * opposite state (true or false), and then sets the piston used for the PTO
 * (on TriPort E) to the new value of ptoActivated (true = extended, false = unextended).
 *
 * This will activate or deactivate the PTO mechanism.
 */
void ptoChange(){
    ptoActivated = !ptoActivated;
    pto.set_value(ptoActivated);
}

/**
 * @brief switches the state of the intake funnel piston.
 *
 * This function first changes the global boolean variable intakeFunnelActivated
 * to its opposite state (true or false), and then sets the piston attached to the
 * intake funnel (on TriPort D) to the new value of intakeFunnelActivated (true = extended,
 * false = unextended).
 *
 * This will activate or deactivate the intake funneling mechanism.
 */
void intakeFunnelChange(){
    intakeFunnelActivated = !intakeFunnelActivated;
    pto.set_value(intakeFunnelActivated);
}

/**
 * @brief switches the state of the double park piston.
 *
 * This function first changes the global boolean variable doubleParkActivated to its
 * opposite state (true or false), and then sets the piston attached to the double park
 * mechanism (on TriPort C) to the new value of doubleParkActivated (true = extended,
 * false = unextended).
 *
 * This will activate or deactivate the double park mechanism.
 */
void doubleParkChange(){
    doubleParkActivated = !doubleParkActivated;
    doublePark.set_value(doubleParkActivated);
}

/**
 * @brief switches the state of the odom lift piston.
 *
 * This function first changes the global boolean variable odomLiftActivated to its
 * opposite state (true or false), and then sets the piston attached to the odom lift
 * (on TriPort B) to the new value of odomLiftActivated (true = extended, false = unextended).
 *
 * This will lift or drop the odometry pod.
 */
void odomLiftChange(){
    odomLiftActivated = !odomLiftActivated;
    odomLift.set_value(odomLiftActivated);
}

/**
 * @brief Translates controller movements to drivetrain movements
 *
 * This function detects up and down movements on the joysticks of the controller
 * and based on how much each joystick is moved up or down, the drivetrain moves accordingly.
 * Down reverses the drivetrain, and up moves the drivetrain forward.
 * The left side of the drive train is controlled by the left joystick, 
 * and the right side of the drive train is controlled by the right joystick.
 * Turning the joysticks in opposite directions turns the bot.
 * If the buttons on the controller are used to move instead, the function
 * moves the drivetrain by a set amount.
 *
 */
void tank(){
    int axisL = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); // Gets amount forward/backward from left joystick
    int axisR = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y); // Gets the turn left/right from right joystick
    
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
        axisL = 127;
        axisR = 127;
    }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        axisL = -127;
        axisR = -127;
    }

    chassis.tank(axisL, axisR); // tank drive
}