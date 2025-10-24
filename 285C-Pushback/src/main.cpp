/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ayaan                                                     */
/*    Created:      10/19/2025, 3:26:09 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;

// define your global instances of motors and other devices here
controller Controller = controller();
int axisLS, axisLT, axisRS, axisRT;

motor Motor1 = motor(PORT1, ratio6_1, false);
motor Motor2 = motor(PORT2, ratio6_1, false);
motor Motor3 = motor(PORT3, ratio6_1, false);
motor Motor4 = motor(PORT11, ratio6_1, true);
motor Motor5 = motor(PORT12, ratio6_1, true);
motor Motor6 = motor(PORT13, ratio6_1, true);
motor_group leftMotors = motor_group(Motor1, Motor2, Motor3);
motor_group rightMotors = motor_group(Motor4, Motor5, Motor6);

motor Intake = motor(PORT20, false);
bool reverseDrive = false;

drivetrain DriveTrain = drivetrain(leftMotors, rightMotors, 260, 285, 252, mm, 48/36);

// shoulder buttons (intake & conveyer)
void r1Pressed(){
    Intake.spin(forward, 200, rpm);
}
void r1Released(){
    Intake.stop();
}
void r2Pressed(){
    Intake.spin(reverse, 200, rpm);
}
void r2Released(){
    Intake.stop();
}

void l2Pressed(){
    reverseDrive = !reverseDrive;
}

// joysticks (tank drive)
/* maybe inlcude gears to make rpm less erratic */
void axisLTChanged(){
    leftMotors.spin(reverseDrive ? reverse : forward, 6 * Controller.Axis3.position(), rpm);
}
void axisRTChanged(){
    rightMotors.spin(reverseDrive ? reverse : forward, 6 * Controller.Axis2.position(), rpm);
}

/*
void axisLSChanged(){
    leftDrive(axisLS);
}
void axisRSChanged(){
    leftDrive(axisRS);
}*/

// main
int main() {
    while(1) {
        //print intake state to screen
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(10, 50);
        Brain.Screen.print(Intake.isSpinning());

        //input (shoulders + joysticks)
        Controller.ButtonR1.pressed(r1Pressed);
        Controller.ButtonR1.released(r1Released);
        Controller.ButtonR2.pressed(r2Pressed);
        Controller.ButtonR2.released(r2Released);
        Controller.ButtonL2.pressed(l2Pressed);

        Controller.Axis3.changed(axisLTChanged);
        Controller.Axis2.changed(axisRTChanged);

        //don't run too many actions
        this_thread::sleep_for(10);
    }
}
