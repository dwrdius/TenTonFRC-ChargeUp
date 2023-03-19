#include "Robot.h"

// Main Controller
frc::XboxController controller{ControllerIDs::kControllerMainID};
//secondary controller
frc::XboxController controllerAux{ControllerIDs::kControllerAuxID};

// Drive Motors
TalonFX FLDriveMotor{CanIDs::kFLDriveMotor};
TalonFX FRDriveMotor{CanIDs::kFRDriveMotor};
TalonFX BLDriveMotor{CanIDs::kBLDriveMotor};
TalonFX BRDriveMotor{CanIDs::kBRDriveMotor};

// Swerve Motors
TalonFX FLSwerveMotor{CanIDs::kFLSwerveMotor};
TalonFX FRSwerveMotor{CanIDs::kFRSwerveMotor};
TalonFX BLSwerveMotor{CanIDs::kBLSwerveMotor};
TalonFX BRSwerveMotor{CanIDs::kBRSwerveMotor};

// arm shooter motors
TalonFX ArmMotor{CanIDs::kArmMotor, "CANCAN"};
TalonFX ShooterTop{CanIDs::kShooterTop, "CANCAN"};
TalonFX ShooterBottom{CanIDs::kShooterBottom, "CANCAN"};

// Intake Falcons
TalonFX IntakeUpDown {CanIDs::kIntakeUpDown, "CANCAN"};

// Encoders
CANCoder FLCANCoder{CanIDs::kFLCANCoder};
CANCoder FRCANCoder{CanIDs::kFRCANCoder};
CANCoder BLCANCoder{CanIDs::kBLCANCoder};
CANCoder BRCANCoder{CanIDs::kBRCANCoder};

//Intake ---------------------------------------------------------------------------------------

// Intake Neo Motors 
rev::CANSparkMax IntakeLeader{RevIDs::kIntakeLeader, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
rev::CANSparkMax IntakeFollower{RevIDs::kIntakeFollower, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

// Gyro
AHRS navX{frc::SPI::kMXP};

// LED
frc::PWMSparkMax LED{RevIDs::kLED};

frc::DigitalInput armLimitSwitch{0};

// Limelight shenanigans
auto table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
double tx;
double ty;

void Robot::RobotInit() 
{
    FLDriveMotor.SetNeutralMode(NeutralMode::Coast);
    FRDriveMotor.SetNeutralMode(NeutralMode::Coast);
    BLDriveMotor.SetNeutralMode(NeutralMode::Coast);
    BRDriveMotor.SetNeutralMode(NeutralMode::Coast);

    // Swerve Motors
    FLSwerveMotor.SetNeutralMode(NeutralMode::Coast);
    FRSwerveMotor.SetNeutralMode(NeutralMode::Coast);
    BLSwerveMotor.SetNeutralMode(NeutralMode::Coast);
    BRSwerveMotor.SetNeutralMode(NeutralMode::Coast);

    // arm shooter motors
    ArmMotor.SetNeutralMode(NeutralMode::Coast);
    ShooterTop.SetNeutralMode(NeutralMode::Coast);
    ShooterBottom.SetNeutralMode(NeutralMode::Coast);
    IntakeUpDown.SetNeutralMode(NeutralMode::Coast);
    
    FLDriveMotor.SetSelectedSensorPosition(0);
    FRDriveMotor.SetSelectedSensorPosition(0);
    BLDriveMotor.SetSelectedSensorPosition(0);
    BRDriveMotor.SetSelectedSensorPosition(0);

    // Swerve Motors
    FLSwerveMotor.SetSelectedSensorPosition(0);
    FRSwerveMotor.SetSelectedSensorPosition(0);
    BLSwerveMotor.SetSelectedSensorPosition(0);
    BRSwerveMotor.SetSelectedSensorPosition(0);

    // arm shooter motors
    ArmMotor.SetSelectedSensorPosition(0);
    ShooterTop.SetSelectedSensorPosition(0);
    ShooterBottom.SetSelectedSensorPosition(0);
    IntakeUpDown.SetSelectedSensorPosition(0);
}
void Robot::RobotPeriodic() 
{
    
    frc::SmartDashboard::PutNumber("Limit", armLimitSwitch.Get());

    frc::SmartDashboard::PutNumber("FLDriveMotor", FLDriveMotor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("FRDriveMotor", FRDriveMotor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("BLDriveMotor", BLDriveMotor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("BRDriveMotor", BRDriveMotor.GetSelectedSensorPosition());

    // Swerve Motors
    frc::SmartDashboard::PutNumber("FLSwerveMotor", FLSwerveMotor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("FRSwerveMotor", FRSwerveMotor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("BLSwerveMotor", BLSwerveMotor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("BRSwerveMotor", BRSwerveMotor.GetSelectedSensorPosition());

    // arm shooter motors
    frc::SmartDashboard::PutNumber("ArmMotor", ArmMotor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("ShooterTop", ShooterTop.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("ShooterBottom", ShooterBottom.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("IntakeUpDown", IntakeUpDown.GetSelectedSensorPosition());
}

void Robot::AutonomousInit() 
{}

void Robot::AutonomousPeriodic() 
{}

void Robot::TeleopInit() 
{}
void Robot::TeleopPeriodic()
{
    if (controller.GetAButton())
    {
        IntakeUpDown.SetNeutralMode(NeutralMode::Brake);
    }
    else
    {
        IntakeUpDown.SetNeutralMode(NeutralMode::Coast);
    }
    if (controller.GetBButton())
    {
        IntakeUpDown.Set(TalonFXControlMode::PercentOutput, -0.07);
    }
}

void Robot::TestInit() {} 
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif