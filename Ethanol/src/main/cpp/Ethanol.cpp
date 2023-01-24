#include <cmath>
#include <iostream>
#include <stdlib.h>

#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include <AHRS.h>
#include <ctre/phoenix/sensors/WPI_CANCoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>

// #include "Constants.h"
using namespace std;

const double rotationVectorMultiplier = 1; // controls how much of the vector addition is dedicated to rotation vs field movement  0 < x < double limit idk
const double speedLimit = 0.2;             // limit motor speed output   0 < x <= 1
const double baseWidth = 14.5;             // inches
const double baseLength = 23.25;

// For determining motor positions on an x,y grid
const double relativeX = baseWidth / 2;
const double relativeY = baseLength / 2;

double relativeAFL;
double getDesiredFR;
double getDesiredBL;
double getDesiredBR;
//pid difference var
double desiredTurnFL;
double distanceFL;
double desiredTurnFR;
double distanceFR;
double desiredTurnBL;
double distanceBL;
double desiredTurnBR;
double distanceBR;
//desired angle
double desiredAFL;
double desiredAFR;
double desiredABL;
double desiredABR;

double driveFL;
double driveFR;
double driveBL;
double driveBR;

double magnitude(double x, double y) // magnitude of vector
{
    return sqrt(x * x + y * y);
}

int magnitudeOptimization(double initialAngle, double finalAngle)
{
    double posDiff = fabs(fmod(finalAngle - initialAngle, M_PI * 2));
    if (posDiff > M_PI_2 && posDiff < M_PI_2 * 3)
    {
        return -1;
    }
    return 1;
}
double angleOptimisation(double initialAngle, double finalAngle)
{
    // return finalAngle;
    double diff = fmod(finalAngle - initialAngle, 360.0);
    double posDiff = fabs(diff);
    if (posDiff > 90.0 && posDiff < 270.0)
    {
        diff = fmod(diff + 180.0, 360.0);
    }
    posDiff = fabs(diff);

    if (posDiff <= 90.0)
    {
        return diff;
    }
    else if (diff >= 270.0)
    {
        return diff - 360.0;
    }
    return diff + 360.0;
}

double findMax(double arr[], int len) // finds max value in array
{
    double Max = arr[0];
    for (int i = 1; i < len; i++)
    {
        Max = max(sus, arr[i]);
    }
    return Max;
}

double getDegree(double x)
{
    double a = x * 180.0 / M_PI;
    return a;
}

double getRadian(double x)
{
    double a = x * M_PI / 180.0;
    return a;
}

double deadband(double joystickInput)
{
    if (abs(joystickInput) <= 0.2)
    {
        return 0.0;
    }
    return joystickInput;
}

// .flm = Front left magnitude, .fla = Front left argument
struct swerveModule
{
    double flm, fla, frm, fra, blm, bla, brm, bra;
    swerveModule(double flm, double fla, double frm, double fra, double blm, double bla, double brm, double bra) : flm(flm), fla(fla), frm(frm), fra(fra), blm(blm), bla(bla), brm(brm), bra(bra) {}
};

struct Point
{
    double x, y;
    Point(double x, double y) : x(x), y(y) {}
};

/*  ali be like:
Point(double x, double y)
{
    this->x = x;
    this->y = y;
}
*/


// this was written in radians but outputs degrees because of an oversight
swerveModule swerveKinematics(double xLeft, double yLeft, double xRight, double gyro)
{
    Point posVector = Point(0.0, 0.0);
    double joystickMagnitude = magnitude(xLeft, yLeft);
    if (joystickMagnitude)
    {
        posVector.x = joystickMagnitude * sin(atan2(xLeft, yLeft) - gyro);
        posVector.y = joystickMagnitude * cos(atan2(xLeft, yLeft) - gyro); // math notation of vector
    }
    else if (abs(xRight) < 0.1)
    {
        return swerveModule(0.0, 1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    double rotationScalar = rotationVectorMultiplier * xRight / magnitude(relativeX, relativeY);

    // declare rotation vector directions and add positional
    Point rawFL = Point((rotationScalar * relativeY) + posVector.x, (rotationScalar * relativeX) + posVector.y);
    Point rawFR = Point((rotationScalar * relativeY) + posVector.x, -(rotationScalar * relativeX) + posVector.y);
    Point rawBL = Point(-(rotationScalar * relativeY) + posVector.x, (rotationScalar * relativeX) + posVector.y);
    Point rawBR = Point(-(rotationScalar * relativeY) + posVector.x, -(rotationScalar * relativeX) + posVector.y);

    // the most sketchy thing ever.
    double magnitudes[4] = {magnitude(rawFL.x, rawFL.y), magnitude(rawFR.x, rawFR.y), magnitude(rawBL.x, rawBL.y), magnitude(rawBR.x, rawBR.y)};
    double limitingScalar = speedLimit / max(1.0, findMax(magnitudes, sizeof(magnitudes) / sizeof(magnitudes[0])));
    Point physFL = Point(limitingScalar * magnitudes[0], atan2(rawFL.x, rawFL.y)); // converts to physics notation
    Point physFR = Point(limitingScalar * magnitudes[1], atan2(rawFR.x, rawFR.y));
    Point physBL = Point(limitingScalar * magnitudes[2], atan2(rawBL.x, rawBL.y));
    Point physBR = Point(limitingScalar * magnitudes[3], atan2(rawBR.x, rawBR.y));
    //-----
    
    return swerveModule(physFL.x, getDegree(physFL.y), physFR.x, getDegree(physFR.y), physBL.x, getDegree(physBL.y), physBR.x, getDegree(physBR.y));
}

class Robot : public frc::TimedRobot
{
    public:
        void TeleopInit() override
        {
            m_FLSwerveMotor.SetInverted(true);
            m_FRSwerveMotor.SetInverted(true);
            m_BLSwerveMotor.SetInverted(true);
            m_BRSwerveMotor.SetInverted(true);

            m_FLDriveMotor.SetNeutralMode(NeutralMode::Brake);
            m_FRDriveMotor.SetNeutralMode(NeutralMode::Brake);
            m_BLDriveMotor.SetNeutralMode(NeutralMode::Brake);
            m_BRDriveMotor.SetNeutralMode(NeutralMode::Brake);

            m_FLSwerveMotor.SetNeutralMode(NeutralMode::Brake);
            m_FRSwerveMotor.SetNeutralMode(NeutralMode::Brake);
            m_BLSwerveMotor.SetNeutralMode(NeutralMode::Brake);
            m_BRSwerveMotor.SetNeutralMode(NeutralMode::Brake);

            pid.EnableContinuousInput(-180, 180);
            pid.SetTolerance(5, 10);

            m_navX.ZeroYaw();

            // encoderTurn.velocityMeasurementPeriod = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_100Ms;
            // encoderTurn.absoluteSensorRange = ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180;
            // encoderTurn.sensorDirection = false;
            // encoderTurn.sensorCoefficient = M_PI / 2048.0;
            // encoderTurn.sensorTimeBase = ctre::phoenix::sensors::SensorTimeBase::PerSecond;
        }
        void TeleopPeriodic() override
        {
            swerveModule moduleDesiredStates = swerveKinematics(deadband(m_Controller.GetLeftX()), deadband(m_Controller.GetLeftY()), deadband(m_Controller.GetRightX()), getRadian(m_navX.GetAngle()), FLCANCoder.GetPosition(), FRCANCoder.GetPosition(), BLCANCoder.GetPosition(), BRCANCoder.GetPosition());
            
            // only update wheel angles when input given
            if (moduleDesiredStates.fla < 600.0)
            {
                driveFL = magnitudeOptimization(FLCANCoder.GetPosition(), moduleDesiredStates.fla)
                desiredAFL = moduleDesiredStates.fla;
            }
            distanceFL = desiredTurnFL + FLCANCoder.GetPosition() - relativeAFL;
            desiredTurnFL = angleOptimisation(FLCANCoder.GetPosition(), desiredAFL);
            relativeAFL = FLCANCoder.GetPosition() + desiredTurnFL;
            m_FLSwerveMotor.Set(ControlMode::Position, pid.Calculate(distanceFL, relativeAFL));

            // distanceFR = desiredTurnFR + FRCANCoder.GetPosition() - relativeAFR;
            // desiredTurnFR = angleOptimisation(FRCANCoder.GetPosition(), desiredAFR);
            // relativeAFR = FRCANCoder.GetPosition() + desiredTurnFR;
            // m_FRSwerveMotor.Set(ControlMode::Position, pid.Calculate(distanceFR, relativeAFR));

            // distanceBL = desiredTurnBL + BLCANCoder.GetPosition() - relativeABL;
            // desiredTurnBL = angleOptimisation(BLCANCoder.GetPosition(), desiredABL);
            // relativeABL = BLCANCoder.GetPosition() + desiredTurnBL;
            // m_BLSwerveMotor.Set(ControlMode::Position, pid.Calculate(distanceBL, relativeABL));

            // distanceBR = desiredTurnBR + BRCANCoder.GetPosition() - relativeABR;
            // desiredTurnBR = angleOptimisation(BRCANCoder.GetPosition(), desiredABR);
            // relativeABR = BRCANCoder.GetPosition() + desiredTurnBR;
            // m_BRSwerveMotor.Set(ControlMode::Position, pid.Calculate(distanceBR, relativeABR));

            frc::SmartDashboard::PutNumber("MFL", driveFL);
            frc::SmartDashboard::PutNumber("MFR", driveFR);
            frc::SmartDashboard::PutNumber("MBL", driveBL);
            frc::SmartDashboard::PutNumber("MBR", driveBR);
            
            frc::SmartDashboard::PutNumber("AFL", desiredAFL);
            frc::SmartDashboard::PutNumber("AFR", moduleDesiredStates.fra);
            frc::SmartDashboard::PutNumber("ABL", moduleDesiredStates.bla);
            frc::SmartDashboard::PutNumber("ABR", moduleDesiredStates.bra);

            frc::SmartDashboard::PutNumber("Distance", diffFL);
            frc::SmartDashboard::PutNumber("Current", FLCANCoder.GetPosition());
            frc::SmartDashboard::PutNumber("Desired", relativeAFL);
            frc::SmartDashboard::PutNumber("Yaw", m_navX.GetAngle());
            frc::SmartDashboard::PutNumber("PID", pid.Calculate(distanceFL, relativeAFL)/*pid.Calculate(diffFL, relativeAFL)*/);
            

            if (m_Controller.GetAButtonPressed())
            {
                m_navX.ZeroYaw();
            }

            // m_FLDriveMotor.Set(ControlMode::PercentOutput, moduleDesiredStates.flm);
            // m_FRDriveMotor.Set(ControlMode::PercentOutput, moduleDesiredStates.frm);
            // m_BLDriveMotor.Set(ControlMode::PercentOutput, moduleDesiredStates.blm);
            // m_BRDriveMotor.Set(ControlMode::PercentOutput, moduleDesiredStates.brm);
        }

    private:
        frc::XboxController m_Controller{0};
        // ctre::phoenix::sensors::CANCoderConfiguration encoderturn;
        frc2::PIDController pid{0.01, 0, 0};

        // Drive Motors
        TalonFX m_FLDriveMotor{3};
        TalonFX m_FRDriveMotor{1};
        TalonFX m_BLDriveMotor{2};
        TalonFX m_BRDriveMotor{4};

        // Swerve Motors
        TalonFX m_FLSwerveMotor{7};
        TalonFX m_FRSwerveMotor{5};
        TalonFX m_BLSwerveMotor{6};
        TalonFX m_BRSwerveMotor{8};

        // Encoders
        CANCoder FLCANCoder{11};
        CANCoder FRCANCoder{9};
        CANCoder BLCANCoder{10};
        CANCoder BRCANCoder{12};

        // Gyro
        AHRS m_navX{frc::SPI::kMXP};
};

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif