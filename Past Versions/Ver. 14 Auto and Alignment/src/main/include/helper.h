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
            break;
        case 2:
            slewRate = 0.1;
    }
    if (abs(diff)>=slewRate)
    {
        desiredPercentage = currentPercentage + slewRate*diff/abs(diff);
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
                input = Deadbands::deadOffset*(input-Deadbands::joyDead*input/fabs(input));
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
        case 3:
            if (abs(input) <= 1)
            {
                input=0;
            }
            break;
        case 4:
            if (abs(input) <= 1000)
            {
                input=0;
            }
            break;
        case 5:
            if (abs(input) <= 0.2)
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
    *swerveState = deadband(slew(*swerveState, swerveCalcs(CANCoder, desiredTurn), 2), 2);
    m_swerve.Set(TalonFXControlMode::PercentOutput, *swerveState);

    // Controls whether the wheels go forwards or backwards depending on the ideal turn angle
    // REMOVE EXPONENT REQUIREMENT AFTER DECIDING ON A CERTAIN EXPONENT
    *driveState = slew(*driveState, driveCalcs(desiredMag, CANCoder, desiredArg), 0);
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

double aprilAlign (double NavX)
    {
        NavX = fmod(NavX, 360.0);
        if (NavX < -180.0 || (NavX < 180.0 && NavX > 0))
        {
            return 90;
        }
        return -90;
    }

// btw everything breaks with an asymmetrical base
void autoRotationScalarFromCoords(double dAngle, double lDisplacement)
{
    mathConst::rotationVectorMultiplier = deadband(((dAngle) / lDisplacement) / mathConst::kDegreesPerInchDenominator, 0);
}

// https://www.desmos.com/calculator/smj51ntx4w
// 0: x^2.5 --> purple
// 1: (x^2 + x^10) / 2 --> blue
// 2: (x^2 + x^5) / 2 --> green
// 3: exp + cubic (high plateau) --> dotted red
// 4: exp + cubic (mid plateau) --> dotted orange
// 5: sigmoid + cubic --> dotted green 

// 6: See Desmos --> https://www.desmos.com/calculator/dw8xhxmm6k
double exponentiate (double magnitude, int type)
{
    switch (type)
    {
        case 0:
            magnitude = pow(magnitude, 2.5);
            break;
        case 1:
            magnitude = (pow(magnitude, 2) + pow(magnitude, 10)) / 2;
            break;
        case 2:
            magnitude = (pow(magnitude, 2) + pow(magnitude, 5)) / 2;
            break;
        case 3:
            if (magnitude<0.3894)
            {
                magnitude = 31*pow(magnitude, 4.75) - 3.74*pow(10, 15) * pow(magnitude, 52.5);
            }
            else
            {
                magnitude = 15.81 * magnitude - 23.28*pow(magnitude, 2) + 11.43*pow(magnitude, 3) - 2.95;
            }
            break;
        case 4:
            if (magnitude<0.2272)
            {
                magnitude = (pow(M_E, 4*(magnitude-0.0065))-1) / 6;
            }
            else
            {
                magnitude = 4.98283*magnitude - 9.08987*pow(magnitude, 2) + 5.6091*pow(magnitude, 3) - 0.492389;
            }
            break;
        case 5:
            if (magnitude<0.5183)
            {
                magnitude = 1 / (2.3 * (1 + pow(M_E, 20*(0.3-magnitude))));
            }
            else
            {
                magnitude = 4.98283*magnitude - 9.08987*pow(magnitude, 2) + 5.6091*pow(magnitude, 3) - 0.492389;
            }
            break;
        case 6:
            if (magnitude < Exponentiation::nexus)
            { 
                magnitude = Exponentiation::arb_a / (1 + pow(M_E, Exponentiation::arb_b * (Exponentiation::arb_c - magnitude))) + Exponentiation::arb_d;
            }
            else
            {
                magnitude = Exponentiation::arb_e * pow(M_E, Exponentiation::arb_f * (magnitude - Exponentiation::arb_g)) + Exponentiation::arb_h + (Exponentiation::arb_a / (1 + pow(M_E, Exponentiation::arb_b * (Exponentiation::arb_c - Exponentiation::nexus))) + Exponentiation::arb_d) - (Exponentiation::arb_e * pow(M_E, Exponentiation::arb_f * (Exponentiation::nexus - Exponentiation::arb_g)) + Exponentiation::arb_h);
            }
            break;
        default:
            break;
    }
    if (magnitude>1)
    {
        magnitude = 1;
    }
    return magnitude;
}

#endif