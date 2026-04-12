#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include "robot/drivetrain.h"
#include "robot/odometry.h"

extern pros::Imu imu;

float getLinearSpeed(){
    float leftRPM = std::abs(left_mg.get_actual_velocity());
    float rightRPM = std::abs(right_mg.get_actual_velocity());
    float avgRPM = (leftRPM + rightRPM) / 2.0f;
    return avgRPM * (M_PI * 3.25) / (60.0f * (36 / 48));
}

float getAngularSpeed(){
    float leftRPM = std::abs(left_mg.get_actual_velocity());
    float rightRPM = std::abs(right_mg.get_actual_velocity());
    float inchesPerRev = M_PI * 3.25 / (36 / 48);
    float leftVel = leftRPM * inchesPerRev / 60.0f;
    float rightVel = rightRPM * inchesPerRev / 60.0f;
    return ((rightVel - leftVel) / 11.22) * (180.0f / M_PI);
}

struct ChainParams{
    float earlyExitRange;
    float minSpeed;
};
struct ChainToPointParams{
    bool forwards = true;
    float maxSpeed = 127;
};
struct ChainToHeadingParams{
    AngularDirection direction = AngularDirection::AUTO;
    int maxSpeed = 127;
};

class CustomChassis : public lemlib::Chassis{
    public:
    using lemlib::Chassis::Chassis;
    
    void chainToPoint(float x, float y, int timeout, ChainToPointParams params = ChainToPointParams(), bool async = true){        
        float currentX = getPose().x;
        float currentY = getPose().y;
        float distance = std::sqrt(std::pow(x - currentX, 2) + std::pow(y - currentY, 2));
        float actualMax = std::min((float) params.maxSpeed, getLinearSpeed() + (1000 * distance));
        float minSpeed = actualMax / 2;
        float earlyExitRange = distance * 15 / 100 * actualMax / 127;

        moveToPoint(x, y, timeout, 
                    {.forwards = params.forwards, 
                     .maxSpeed = params.maxSpeed,
                     .minSpeed = minSpeed, 
                     .earlyExitRange = earlyExitRange}, 
                    async);
    }

    void chainToHeading(float theta, int timeout, ChainToHeadingParams params = ChainToHeadingParams(), bool async = true){
        float currentHeading = getPose().theta;
        float distance = std::abs(currentHeading - theta);
        float actualMax = std::min((float) params.maxSpeed, getAngularSpeed() + (500 * distance));
        int minSpeed = actualMax / 2;
        float earlyExitRange = distance * 8 / 100 * actualMax / 127;

        turnToHeading(theta, timeout,
                      {.direction = params.direction,
                       .maxSpeed = params.maxSpeed},
                      async);
    }
};

extern CustomChassis chassis;