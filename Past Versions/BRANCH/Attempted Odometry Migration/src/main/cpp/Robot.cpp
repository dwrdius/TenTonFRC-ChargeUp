#include "Robot.h"

// Proportional value given to turn motors
// Desired turn angles; it's multipurpose to remove clutter :p
double desiredTurnFL;
double desiredTurnFR;
double desiredTurnBL;
double desiredTurnBR;

// Intermediary Variable for implementing slew to swerve motors
double FLSwerveState;
double FRSwerveState;
double BLSwerveState;
double BRSwerveState;

// Intermediary Variable for implementing slew to drive motors
double FLDriveState;
double FRDriveState;
double BLDriveState;
double BRDriveState;

// units::length::inch_t falconFXToInches(double selectedSensorPosition) {
//   return units::length::inch_t{(selectedSensorPosition * 4 * M_PI) / 180};
// }

// all doubles
// order: front left magnitude, front left angle, frm, fra, blm, bla, brm, bra
// percentage magnitude
// degree angle
struct desiredSwerveModule
{
    double flm, fla, frm, fra, blm, bla, brm, bra;
    desiredSwerveModule(double flm, double fla, double frm, double fra, double blm, double bla, double brm, double bra) : flm(flm), fla(fla), frm(frm), fra(fra), blm(blm), bla(bla), brm(brm), bra(bra) {}
};

// Stores a vector
struct Point
{
    double x, y;
    Point(double x, double y) : x(x), y(y) {}
};

// Input degrees
// Automatically applies deadbands
// Outputs a swerveModule object 
// (percentage speeds and angles in the order FL, FR, BL, BR)
desiredSwerveModule swerveKinematics(double xLeft, double yLeft, double xRight, double gyro)
{
    // cmath reads radians; applies deadbands
    gyro = getRadian(gyro);
    xLeft = deadband(xLeft);
    yLeft = deadband(yLeft);
    xRight = deadband(xRight);
    
    // Creates a unit vector multiplied by right joystick input and proportionally scaled by "rotationVectorMultiplier"
    double rotationScalar = -xRight;

    Point posVector = Point(0.0, 0.0);
    double joystickMagnitude = pow(magnitude(xLeft, yLeft), mathConst::driveExponent);
    if (joystickMagnitude)  // Check if left joystick has an input
    {
        // atan2 converts joystick input into angle
        // + gyro makes desired angle calculation field relative 
        // (was -gyro in math but +gyro works in practice?)
        double fieldRelativePosAngle = atan2(xLeft, yLeft) + gyro;
        
        // convert angles back into Cartesian
        posVector.x = joystickMagnitude * sin(fieldRelativePosAngle);
        posVector.y = joystickMagnitude * cos(fieldRelativePosAngle);
        rotationScalar = mathConst::rotationVectorMultiplier * rotationScalar; 
    }
    else if (abs(xRight) < 0.1) // the < 0.1 is another reduncancy that is within the deadband
    {
        // if no joystick input, return exit code
        return desiredSwerveModule(0.0, 1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    // Apply rotation vectors to positional vectors to create the combined vector
    // negatives and "y, x" are assigned (negative reciprocal = perpendicular line)
    // actually the y values were negated again because it just worked. I don't know if I did math wrong, but it worked so there's another arbitrary negation
    Point rawFL = Point((rotationScalar * mathConst::xCoords[0]) + posVector.x, (rotationScalar * mathConst::yCoords[0]) + posVector.y);
    Point rawFR = Point((rotationScalar * mathConst::xCoords[1]) + posVector.x, (rotationScalar * mathConst::yCoords[1]) + posVector.y);
    Point rawBL = Point((rotationScalar * mathConst::xCoords[2]) + posVector.x, (rotationScalar * mathConst::yCoords[2]) + posVector.y);
    Point rawBR = Point((rotationScalar * mathConst::xCoords[3]) + posVector.x, (rotationScalar * mathConst::yCoords[3]) + posVector.y);

    // compares magnitudes of resulting vectors to see if any composite vector (rotation + position) exceeded "100%" output speed. 
    // Divide by largest value greater than 100% to limit all magnitudes to 100% at max whilst maintaining relative rotational speeds
    // Limiting Scalar also applies the motor speed limit cap
    double magnitudes[4] = {magnitude(rawFL.x, rawFL.y), magnitude(rawFR.x, rawFR.y), magnitude(rawBL.x, rawBL.y), magnitude(rawBR.x, rawBR.y)};
    double limitingScalar = findMax(magnitudes, sizeof(magnitudes) / sizeof(magnitudes[0]));
    
    // Convert to Polar vectors for speed and direction for swerve modules
    Point physFL = Point(magnitudes[0]/limitingScalar, atan2(rawFL.x, rawFL.y));
    Point physFR = Point(magnitudes[1]/limitingScalar, atan2(rawFR.x, rawFR.y));
    Point physBL = Point(magnitudes[2]/limitingScalar, atan2(rawBL.x, rawBL.y));
    Point physBR = Point(magnitudes[3]/limitingScalar, atan2(rawBR.x, rawBR.y));

    return desiredSwerveModule(physFL.x, getDegree(physFL.y), physFR.x, getDegree(physFR.y), physBL.x, getDegree(physBL.y), physBR.x, getDegree(physBR.y));
}
void Robot::RobotInit()
{
    processBaseDimensions(mathConst::xCoords, mathConst::yCoords);
    
    initializeAllComponents();

    desiredTurnFL=FLCANCoder.GetAbsolutePosition();
    desiredTurnFR=FRCANCoder.GetAbsolutePosition();
    desiredTurnBL=BLCANCoder.GetAbsolutePosition();
    desiredTurnBR=BRCANCoder.GetAbsolutePosition();
}


void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{
    // moduleDesiredStates contains all calculated desired values.
    // .flm is "Front left magnitude" (percentage)
    // .fla is "Front left angle" (degrees)
    // fl[], fr[], bl[], br[]
    desiredSwerveModule moduleDesiredStates = swerveKinematics(m_Controller.GetLeftX(), m_Controller.GetLeftY(), m_Controller.GetRightX(), navX.GetAngle());
    
    // when controller joysticks have no input, fla is 1000.0 (pseudo exit code)
    // this only updates the "desired angle" read by the turn motors if the exit code is not detected
    // 600 is an arbitrary value that is over a full rotation less than 1000 for reduncancy; anything 90<x<1000 works
    if (moduleDesiredStates.fla < 600.0)
    {
        // Update wheel angles for the turn motors to read
        desiredTurnFL = moduleDesiredStates.fla;
        desiredTurnFR = moduleDesiredStates.fra;
        desiredTurnBL = moduleDesiredStates.bla;
        desiredTurnBR = moduleDesiredStates.bra;
    }

    setDesiredState(m_FLSwerveMotor, m_FLDriveMotor, &FLSwerveState, FLCANCoder.GetAbsolutePosition(), desiredTurnFL, &FLDriveState, moduleDesiredStates.flm, moduleDesiredStates.fla);
    setDesiredState(m_FRSwerveMotor, m_FRDriveMotor, &FRSwerveState, FRCANCoder.GetAbsolutePosition(), desiredTurnFR, &FRDriveState, moduleDesiredStates.frm, moduleDesiredStates.fra);
    setDesiredState(m_BLSwerveMotor, m_BLDriveMotor, &BLSwerveState, BLCANCoder.GetAbsolutePosition(), desiredTurnBL, &BLDriveState, moduleDesiredStates.blm, moduleDesiredStates.bla);
    setDesiredState(m_BRSwerveMotor, m_BRDriveMotor, &BRSwerveState, BRCANCoder.GetAbsolutePosition(), desiredTurnBR, &BRDriveState, moduleDesiredStates.brm, moduleDesiredStates.bra);
    
// Debug Math Outputs
    // Drive motor speeds (percentage)
    logSwerveNumber("Magnitude", FLDriveState, FRDriveState, BLDriveState, BRDriveState);
    
    // Desired turn angles (degrees)
    logSwerveNumber("Desired Angle", desiredTurnFL, desiredTurnFR, desiredTurnBL, desiredTurnBR);

    // CANCoder Absolute Readings
    logSwerveNumber("CANCoder", FLCANCoder.GetAbsolutePosition(), FRCANCoder.GetAbsolutePosition(), BLCANCoder.GetAbsolutePosition(), BRCANCoder.GetAbsolutePosition());

    frc::SmartDashboard::PutNumber("RightJoy", deadband(m_Controller.GetRightX()));
    
    // Gyro angle (degrees)
    frc::SmartDashboard::PutNumber("Yaw", navX.GetAngle());

// --------- this section is for testing; kenta chooses which features stay and the trigger things are only for configuring preference
    // Zero gyro (press d-pad in whatever direction the PDP is relative to the North you want)
    if (m_Controller.GetPOV()!=-1)
    {
        navX.ZeroYaw();
        navX.SetAngleAdjustment(m_Controller.GetPOV());
    }
// ----------------------
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif