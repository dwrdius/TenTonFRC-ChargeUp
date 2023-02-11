

#ifndef HELPERS_H
#include <cmath>
#include "constants.h"
#include <ctre/Phoenix.h>




// magnitude of vector to convert from Cartesian to Polar
double magnitude(double x, double y) 
{
    return sqrt(x * x + y * y);
}

// depending on the angles, the optimal turn may reverse motor direction
// Returns 1 or -1 based on optimal turn angle
// Takes initial and desired angle
int magnitudeOptimization(double initialAngle, double finalAngle)
{
    double posDiff = fabs(fmod(finalAngle - initialAngle, 360.0));
    if (posDiff > 90.0 && posDiff < 270.0)
    {
        return -1;
    }
    return 1;
}

// ideally, the wheel will only ever turn 90 degrees clockwise or counterclockwise and change drive motor direction to reach desired angle
// Returns optimal turn angle based 
// Takes initial and desired angle
double angleOptimisation(double initialAngle, double finalAngle)
{
    double diff = fmod(finalAngle - initialAngle, 360.0);
    double posDiff = fabs(diff);
    if (posDiff > 90.0 && posDiff < 270.0)
    {
        diff = fmod(diff + 180.0, 360.0);
    }
    if (fabs(diff) <= 90.0)
    {
        return diff;
    }
    else if (diff >= 270.0)
    {
        return diff - 360.0;
    }
    return diff + 360.0;
}

// Given an array of values, find the largest member greater than 1
// if all members are less than one, return 1
// else, return largest value
double findMax(double arr[], int len) // finds max value in array
{
    double Max = 1.0;
    for (int i = 0; i < len; i++)
    {
        Max = fmax(Max, arr[i]);
    }
    return Max;
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

double slew(double currentPercentage, double desiredPercentage)
{
    double diff = desiredPercentage-currentPercentage;
    double slewRate = mathConst::slew;
    if (abs(diff)>=slewRate)
    {
        desiredPercentage = currentPercentage - slewRate*2*(std::signbit(diff)-0.5);
    }
    return desiredPercentage;
}

double swerveDeadband(double swerveTurn)
{
    if (abs(swerveTurn) <= mathConst::swerveDeadband)
    {
        return 0.0;
    }
    return swerveTurn;
}

// Deadband for joystick
// prevents drift at values close to 0
// Else converts range from deadband-1 to 0-1
// 0: Joystick, 1: LL Distance
double deadband(double input, int mode)
{
    switch (mode){
        case 0:
            if (abs(input) <= mathConst::deadband)
            {
                input=0;
            }
            else
            {
                input = mathConst::deadbandOffset*(input+mathConst::deadband*2*(std::signbit(input)-0.5));
            }
            break;
        case 1:
            if (abs(input) <= Limelight::LLDistanceDead)
            {
                input=0;
            }
            break;
    }
    return input;
}

double driveCalcs(double magnitude, double CANCoder, double angle)
{
    return mathConst::speedLimit*magnitude*magnitudeOptimization(CANCoder, angle);
}

double swerveCalcs(double CANCoder, double desiredAngle)
{
    return mathConst::swerveMotorSpeed/90.0*angleOptimisation(CANCoder, desiredAngle);
}

void setDesiredState(TalonFX& m_swerve, TalonFX& m_drive, double *swerveState, double CANCoder, double desiredTurn, double *driveState, double desiredMag, double desiredArg)
{
    // convert desired angle to optimal turn angle and divide by 90 degrees to convert to percentage
            // limit motor turn speed
    *swerveState = swerveDeadband(slew(*swerveState, swerveCalcs(CANCoder, desiredTurn)));
    m_swerve.Set(TalonFXControlMode::PercentOutput, *swerveState);

    // Controls whether the wheels go forwards or backwards depending on the ideal turn angle
    // REMOVE EXPONENT REQUIREMENT AFTER DECIDING ON A CERTAIN EXPONENT
    *driveState = slew(*driveState, driveCalcs(desiredMag, CANCoder, desiredArg));
    m_drive.Set(ControlMode::PercentOutput, *driveState);
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

double demonicslew(double currentPercentage, double desiredPercentage)
{
    double diff = desiredPercentage-currentPercentage;
    double slewRate = 0.02;
    if (abs(diff)>=slewRate)
    {
        desiredPercentage = currentPercentage - slewRate*2*(std::signbit(diff)-0.5);
    }
    return desiredPercentage;
}
#endif