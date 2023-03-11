#ifndef DEVICE_MANAGER_H
#define DEVICE_MANAGER_H

// motors
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>

// xbox, navx (gyro), cancoder (encoder)
#include <frc/XboxController.h>
#include <AHRS.h>
#include <ctre/phoenix/sensors/WPI_CANCoder.h>

// LED
// https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf for colour config
#include <frc/motorcontrol/PWMSparkMax.h>

// Colour
#include <rev/ColorSensorV3.h>


#include <frc/controller/PIDController.h>

namespace ControllerIDs {
    //USB port addresses on drivestation PC.
    constexpr int kControllerMainID = 0;
    constexpr int kControllerAuxID = 1;
}

namespace CanIDs {
    // Drive Motors
    const int kFLDriveMotor = 3;
    const int kFRDriveMotor = 1;
    const int kBLDriveMotor = 2;
    const int kBRDriveMotor = 4;

    // Swerve Motors
    const int kFLSwerveMotor = 7;
    const int kFRSwerveMotor = 5;
    const int kBLSwerveMotor = 6;
    const int kBRSwerveMotor = 8;

    // Encoders
    const int kFLCANCoder = 11;
    const int kFRCANCoder = 9;
    const int kBLCANCoder = 10;
    const int kBRCANCoder = 12;

    // CANCAN CANivore    
    //Intake Talon
    const int kIntakeUpDown = 1;

    //Arm talons
    const int kArmMotor = 2;
    const int kShooterBottom = 3;
    const int kShooterTop = 4;

}

namespace RevIDs {
// TEMP IDs -------------------------------------------------------------------------------------------------------------------------

    // Intake Motors Neo
    const int kIntakeLeader = 14;
    const int kIntakeFollower = 13;

    // LED
    const int kLED = 1;    
}

// Main Controller
extern frc::XboxController controller;
//secondary controller
extern frc::XboxController controllerAux;

// Drive Motors
extern TalonFX FLDriveMotor;
extern TalonFX FRDriveMotor;
extern TalonFX BLDriveMotor;
extern TalonFX BRDriveMotor;

// Swerve Motors
extern TalonFX FLSwerveMotor;
extern TalonFX FRSwerveMotor;
extern TalonFX BLSwerveMotor;
extern TalonFX BRSwerveMotor;

// arm shooter motors
extern TalonFX ArmMotor;
extern TalonFX ShooterTop;
extern TalonFX ShooterBottom;

// Intake Falcons
extern TalonFX IntakeUpDown;

// Encoders
extern CANCoder FLCANCoder;
extern CANCoder FRCANCoder;
extern CANCoder BLCANCoder;
extern CANCoder BRCANCoder;

//Intake ---------------------------------------------------------------------------------------

// Intake Neo Motors 
extern rev::CANSparkMax IntakeLeader;
extern rev::CANSparkMax IntakeFollower;

// Gyro
extern AHRS navX;

// LED
extern frc::PWMSparkMax LED;

// Colour Sensor
extern rev::ColorSensorV3 m_colorSensor;

// PIDs
extern frc2::PIDController autoCoordPID;
extern frc2::PIDController alignAnglePID;

#endif