#ifndef HELPERS_H
#include <cmath>
#include "constants.h"
#include <ctre/Phoenix.h>

// magnitude of vector to convert from Cartesian to Polar
double magnitude(double x, double y) 
{
    return sqrt(x * x + y * y);
}

// Convert radian to degree
double getDegree(double x)
{
    return x * 180.0 / M_PI;
}

// Convert degree to radian
double getRadian(double x)
{
    return x * M_PI / 180.0;
}

// Modes: 0 = drive; 1 = limelight
double slew(double currentPercentage, double desiredPercentage, int mode)
{
    double diff = desiredPercentage-currentPercentage;
    double slewRate=0;
    switch (mode) {
        case 0:
            slewRate = Slews::driveSlew;
            break;
        case 1:
            slewRate = Slews::LLSlew;
    }
    if (abs(diff)>=slewRate)
    {
        desiredPercentage = currentPercentage - slewRate*2*(std::signbit(diff)-0.5);
    }
    return desiredPercentage;
}

// prevents drift at values close to 0
// Else converts range from deadband-1 to 0-1
// 0: Joystick, 1: LL Distance, 2: swerve angle
double deadband(double input, int mode)
{
    switch (mode){
        case 0:
            if (abs(input) <= Deadbands::joyDead)
            {
                input=0;
            }
            else
            {
                input = Deadbands::deadOffset*(input+Deadbands::joyDead*2*(std::signbit(input)-0.5));
            }
            break;
        case 1:
            if (abs(input) <= Deadbands::LLDistanceDead)
            {
                input=0;
            }
            break;
        case 2:
            if (abs(input) <= Deadbands::swerveDeadband)
            {
                input=0;
            }
            break;
    }
    return input;
}

// 0 for x coords; 1 for y coords. 
// Converts to reciprocal unit vector
void processBaseDimensions (double *xCoords, double *yCoords)
{
    double intermediate;
    double mag_int;
    for (int i = 0; i < 4; i++){
        mag_int = sqrt(xCoords[i]*xCoords[i] + yCoords[i]*yCoords[i]);
        intermediate = yCoords[i] / mag_int;
        yCoords[i] = xCoords[i] / mag_int;
        xCoords[i] = intermediate;
    }
}

// This is for the autonomous to balance on the charging station
double getAutoBalanceVelocity(double currentRoll){
    const double rollToVelocityConst = -0.01; // random constant (needs adjusting)
    double balanceVelocity; // declare velocity variable
     
    //make a deadzone so the robot isn't always adjusting
    if (abs(currentRoll)<= 3){
        balanceVelocity = 0;
    }
    else{
      balanceVelocity = currentRoll * rollToVelocityConst; //multiply the navX roll by a constant to get a velocity value
    }
    return balanceVelocity;
}

// btw everything breaks with an asymmetrical base
void autoRotationScalarFromCoords(double dAngle, double lDisplacement)
{
    mathConst::rotationVectorMultiplier = ((dAngle) / lDisplacement) / mathConst::kDegreesPerInchDenominator;
}



#endif