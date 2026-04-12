#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include "robot/drivetrain.h"
#include "robot/odometry.h"

extern pros::Imu imu;

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
        float actualMax = std::min((float) params.maxSpeed, 1000 * distance);
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
        float actualMax = std::min((float) params.maxSpeed, 500 * distance);
        int minSpeed = actualMax / 2;
        float earlyExitRange = distance * 8 / 100 * actualMax / 127;

        turnToHeading(theta, timeout,
                      {.direction = params.direction,
                       .maxSpeed = params.maxSpeed},
                      async);
    }
};

extern CustomChassis chassis;