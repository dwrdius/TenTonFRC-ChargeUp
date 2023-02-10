#include "DeviceManager.h"

// Initialize all components

// Main Controller
frc::XboxController m_Controller{ControllerIDs::kControllerMainID};

// Drive Motors
TalonFX m_FLDriveMotor{CanIDs::kFLDriveMotor};
TalonFX m_FRDriveMotor{CanIDs::kFRDriveMotor};
TalonFX m_BLDriveMotor{CanIDs::kBLDriveMotor};
TalonFX m_BRDriveMotor{CanIDs::kBRDriveMotor};

// Swerve Motors
TalonFX m_FLSwerveMotor{CanIDs::kFLSwerveMotor};
TalonFX m_FRSwerveMotor{CanIDs::kFRSwerveMotor};
TalonFX m_BLSwerveMotor{CanIDs::kBLSwerveMotor};
TalonFX m_BRSwerveMotor{CanIDs::kBRSwerveMotor};

// Encoders
CANCoder FLCANCoder{CanIDs::kFLCANCoder};
CANCoder FRCANCoder{CanIDs::kFRCANCoder};
CANCoder BLCANCoder{CanIDs::kBLCANCoder};
CANCoder BRCANCoder{CanIDs::kBRCANCoder};

// Gyro
AHRS navX{frc::SPI::kMXP};

// config
void initializeAllComponents() {
    m_FLDriveMotor.SetNeutralMode(NeutralMode::Brake);
    m_FRDriveMotor.SetNeutralMode(NeutralMode::Brake);
    m_BLDriveMotor.SetNeutralMode(NeutralMode::Brake);
    m_BRDriveMotor.SetNeutralMode(NeutralMode::Brake);

    m_FLSwerveMotor.SetNeutralMode(NeutralMode::Brake);
    m_FRSwerveMotor.SetNeutralMode(NeutralMode::Brake);
    m_BLSwerveMotor.SetNeutralMode(NeutralMode::Brake);
    m_BRSwerveMotor.SetNeutralMode(NeutralMode::Brake);

    navX.ZeroYaw();

    FLCANCoder.ConfigAbsoluteSensorRange(Signed_PlusMinus180);
    FRCANCoder.ConfigAbsoluteSensorRange(Signed_PlusMinus180);
    BLCANCoder.ConfigAbsoluteSensorRange(Signed_PlusMinus180);
    BRCANCoder.ConfigAbsoluteSensorRange(Signed_PlusMinus180);

    m_FLDriveMotor.ConfigPeakOutputForward(mathConst::speedLimit);
    m_FRDriveMotor.ConfigPeakOutputForward(mathConst::speedLimit);
    m_BLDriveMotor.ConfigPeakOutputForward(mathConst::speedLimit);
    m_BRDriveMotor.ConfigPeakOutputForward(mathConst::speedLimit);

    m_FLDriveMotor.ConfigPeakOutputReverse(-mathConst::speedLimit);
    m_FRDriveMotor.ConfigPeakOutputReverse(-mathConst::speedLimit);
    m_BLDriveMotor.ConfigPeakOutputReverse(-mathConst::speedLimit);
    m_BRDriveMotor.ConfigPeakOutputReverse(-mathConst::speedLimit);

    FLCANCoder.ConfigMagnetOffset(CANCoderOffsets::kFrontLeft);
    FRCANCoder.ConfigMagnetOffset(CANCoderOffsets::kFrontRight);
    BLCANCoder.ConfigMagnetOffset(CANCoderOffsets::kBackLeft);
    BRCANCoder.ConfigMagnetOffset(CANCoderOffsets::kBackRight);
}
