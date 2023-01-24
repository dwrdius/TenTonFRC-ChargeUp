#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include "Constants.h"
#include <AHRS.h>
#include <ctre/phoenix/sensors/WPI_CANCoder.h>


using namespace std;

const double rotationVectorMultiplier = 1;   // controls how much of the vector addition is dedicated to rotation vs field movement  0 < x < double limit idk
const double speedLimit = 0.2;   // limit motor speed output   0 < x <= 1
const double baseWidth = 14.5;   // inches
const double baseLength = 23.25;

// For determining motor positions on an x,y grid
const double relativeX = baseWidth / 2;
const double relativeY = baseLength / 2;

double magnitude(double x, double y) // magnitude of vector
{
    return sqrt(x*x + y*y);
}

int magnitudeOptimization(double initialAngle, double finalAngle)
{
    double posDiff = fabs(fmod(finalAngle-initialAngle, M_PI*2));
    if(posDiff>M_PI_2 && posDiff<M_PI_2*3){
        return -1;
    }
    return 1;
}
double angleOptimisation(double initialAngle, double finalAngle)
{
    double diff = fmod(finalAngle-initialAngle, M_PI*2);
    double posDiff = fabs(diff);
    if(posDiff>M_PI_2 && posDiff<M_PI_2*3){
        diff = fmod(diff + M_PI, M_PI*2);
    }
    posDiff = fabs(diff);
    
    if(posDiff<=M_PI_2){
        return diff;
    }
    else if(diff>=3*M_PI_2){
        return diff-(2*M_PI);
    }
    else{
        return diff+(2*M_PI);
    }
}

double findMax(double arr[], int a) // finds max value in array
{
    double sus = max(arr[0],arr[1]);
    for(int i=2; i<a; i++){
        sus = max(sus,arr[i]);
    }
    return sus;
}

int getDegree(double x)
{
    double a = x*180/M_PI;
    int b = static_cast<int>(a);
    return b;
}

double swerveKinematics(double xLeft, double yLeft, double xRight, double gyro, double FLTheta, double FRTheta, double BLTheta, double BRTheta)
{
    double posVect[2];
    if(magnitude(xLeft, yLeft)){
        posVect[0] = magnitude(xLeft,yLeft)*sin(atan2(xLeft,yLeft)-gyro);
        posVect[1] = magnitude(xLeft,yLeft)*cos(atan2(xLeft,yLeft)-gyro); // math notation of vector
    }
    else{
        if(xRight<0.1){
            double swerveNumbers[4][2] = {{0.0,10.0}, {0.0,0.0}, {0.0,0.0}, {0.0,0.0}};
            return ( swerveNumbers );
        }
        posVect[0] = 0;
        posVect[1] = 0;
    }
    
    double rotationScalar = rotationVectorMultiplier*xRight/magnitude(relativeX,relativeY);
    
    // declare rotation vector directions and add positional
    double frontLeft[2] = {(rotationScalar*relativeY)+posVect[0], (rotationScalar*relativeX)+posVect[1]};
    double frontRight[2] = {(rotationScalar*relativeY)+posVect[0], -(rotationScalar*relativeX)+posVect[1]};
    double backLeft[2] = {-(rotationScalar*relativeY)+posVect[0], (rotationScalar*relativeX)+posVect[1]};
    double backRight[2] = {-(rotationScalar*relativeY)+posVect[0], -(rotationScalar*relativeX)+posVect[1]}; 
    
    // the most sketchy thing ever.
    double magnitudes[4] = {magnitude(frontLeft[0], frontLeft[1]), magnitude(frontRight[0], frontRight[1]), magnitude(backLeft[0], backLeft[1]), magnitude(backRight[0], backRight[1])};
    double limitingScalar = speedLimit/max(1.0, findMax(magnitudes, sizeof(magnitudes)/sizeof(magnitudes[0])));

    double physFL[2] = {limitingScalar*magnitudes[0], atan2(frontLeft[0], frontLeft[1])}; // converts to physics notation
    double physFR[2] = {limitingScalar*magnitudes[1], atan2(frontRight[0], frontRight[1])};
    double physBL[2] = {limitingScalar*magnitudes[2], atan2(backLeft[0], backLeft[1])};
    double physBR[2] = {limitingScalar*magnitudes[3], atan2(backRight[0], backRight[1])};
    //-----
    double FL[2] = {magnitudeOptimization(FLTheta, physFL[1])*physFL[0], angleOptimisation(FLTheta, physFL[1])};
    double FR[2] = {magnitudeOptimization(FRTheta, physFR[1])*physFR[0], angleOptimisation(FRTheta, physFR[1])};
    double BL[2] = {magnitudeOptimization(BLTheta, physBL[1])*physBL[0], angleOptimisation(BLTheta, physBL[1])};    
    double BR[2] = {magnitudeOptimization(BRTheta, physBR[1])*physBR[0], angleOptimisation(BRTheta, physBR[1])};

    //*** [][0] = [Module][motor strength]; [][1] = [Module][angle to turn]
    //[Module] order: [0,1,2,3] = [FL,FR,BL,BR]
    double *swerveNumbers[4] = {FL, FR, BL, BR};
    return ( swerveNumbers );
}

double deadband(double joystickInput)
{
  if(abs(joystickInput)<=0.1){
    return 0.0;
  }
  return joystickInput;
}
class Robot : public frc::TimedRobot {
 public:
  void TeleopInit() override {
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

    encoderTurn.velocityMeasurementPeriod = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_100Ms;
    encoderTurn.absoluteSensorRange = ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180;
    encoderTurn.sensorDirection = false;
    encoderTurn.sensorCoefficient = M_PI / 2048.0;
    encoderTurn.sensorTimeBase = ctre::phoenix::sensors::SensorTimeBase::PerSecond
  }
  void TeleopPeriodic() override {
    double *moduleDesiredStates = swerveKinematics(deadband(m_Controller.GetLeftX()), m_Controller.GetLeftY(), m_Controller.GetRightX(), m_navX.GetRotation2d(), FLTheta, FRTheta, BLTheta, BRTheta)
    if(moduleDesiredStates[0][1]<8){
          m_FLSwerveMotor.Set(ctre::phoenix::sensors::motorcontrol::TalonFXControlMode::Position, moduleDesiredStates[0][1]);
          m_FRSwerveMotor.Set(ctre::phoenix::sensors::motorcontrol::TalonFXControlMode::Position, moduleDesiredStates[1][1]);
          m_BLSwerveMotor.Set(ctre::phoenix::sensors::motorcontrol::TalonFXControlMode::Position, moduleDesiredStates[2][1]);
          m_BRSwerveMotor.Set(ctre::phoenix::sensors::motorcontrol::TalonFXControlMode::Position, moduleDesiredStates[3][1]);
    }
    m_FLDriveMotor.Set(ControlMode::PercentOutput, moduleDesiredStates[0][0]);
    m_FRDriveMotor.Set(ControlMode::PercentOutput, moduleDesiredStates[1][0]);
    m_BLDriveMotor.Set(ControlMode::PercentOutput, moduleDesiredStates[2][0]);
    m_BRDriveMotor.Set(ControlMode::PercentOutput, moduleDesiredStates[3][0]);
   }

 private:
  frc::XboxController m_Controller{0};
  ctre::phoenix::sensors::CANCoderConfiguration encoderturn;

  // Drive Motors
  TalonFX m_FLDriveMotor{drivetrainConstants::kMotorDriveFL};
  TalonFX m_FRDriveMotor{drivetrainConstants::kMotorDriveFR};
  TalonFX m_BLDriveMotor{drivetrainConstants::kMotorDriveBL};
  TalonFX m_BRDriveMotor{drivetrainConstants::kMotorDriveBR};
  
  // Swerve Motors
  TalonFX m_FLSwerveMotor{drivetrainConstants::kMotorTurnFL};
  TalonFX m_FRSwerveMotor{drivetrainConstants::kMotorTurnFR};
  TalonFX m_BLSwerveMotor{drivetrainConstants::kMotorTurnBL};
  TalonFX m_BRSwerveMotor{drivetrainConstants::kMotorTurnBR};

  // Encoders
  CANCoder FLCANCoder{drivetrainConstants::kEncoderTurnFL};
  CANCoder FRCANCoder{drivetrainConstants::kEncoderTurnFR};
  CANCoder BLCANCoder{drivetrainConstants::kEncoderTurnBL};
  CANCoder BRCANCoder{drivetrainConstants::kEncoderTurnBR};

  // Gyro
  AHRS m_navX{frc::SPI::kMXP};
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif