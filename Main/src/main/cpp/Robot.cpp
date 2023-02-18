#include "Robot.h"

// Initialize devices.

// Main Controller
frc::XboxController controller{ControllerIDs::kControllerMainID};

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

// Encoders
CANCoder FLCANCoder{CanIDs::kFLCANCoder};
CANCoder FRCANCoder{CanIDs::kFRCANCoder};
CANCoder BLCANCoder{CanIDs::kBLCANCoder};
CANCoder BRCANCoder{CanIDs::kBRCANCoder};

// Gyro
AHRS navX{frc::SPI::kMXP};

// wall of not constant variable shame 
// Limelight shenanigans
auto table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
double tx;
double ty;

bool pipeline = 1;

double angleToGoalDegrees;
double distanceFromLimelightToGoalInches;

double LimelightDifference=1;
double LimelightSlew;
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

// debugging rotational rpm
double initAngle;
double degreesofRotation;

bool goBalanceDog = true;

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
    
    // Creates a unit vector multiplied by right joystick input and proportionally scaled by "rotationVectorMultiplier"
    double rotationScalar = xRight;

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

desiredSwerveModule moduleDesiredStates = swerveKinematics(0, 0, 0, 0);

frc::SwerveDriveKinematics<4> kinematics{
  Odometry::kFLLocation, 
  Odometry::kFRLocation,
  Odometry::kBLLocation, 
  Odometry::kBRLocation
};

// Convert SelectedSensorPosition (degrees) to Inches.
units::length::inch_t TalonFXToInches(double selectedSensorPosition) {
  return units::length::inch_t{(selectedSensorPosition * 4 * M_PI) / 180};
}

// Creating odometry object from the kinematics object, navX rotation, swerve module positions.
frc::SwerveDriveOdometry<4> odometry{
  kinematics,
  navX.GetRotation2d(),
  {
    frc::SwerveModulePosition{TalonFXToInches(FLDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FLCANCoder.GetPosition()})}, 
    frc::SwerveModulePosition{TalonFXToInches(FRDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FRCANCoder.GetPosition()})}, 
    frc::SwerveModulePosition{TalonFXToInches(BLDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BLCANCoder.GetPosition()})}, 
    frc::SwerveModulePosition{TalonFXToInches(BRDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BRCANCoder.GetPosition()})}
  },
  frc::Pose2d{0_m, 0_m, 0_deg}
};

void Robot::RobotInit() 
{
    processBaseDimensions(mathConst::xCoords, mathConst::yCoords);

    FLDriveMotor.SetNeutralMode(NeutralMode::Brake);
    FRDriveMotor.SetNeutralMode(NeutralMode::Brake);
    BLDriveMotor.SetNeutralMode(NeutralMode::Brake);
    BRDriveMotor.SetNeutralMode(NeutralMode::Brake);

    FLSwerveMotor.SetNeutralMode(NeutralMode::Brake);
    FRSwerveMotor.SetNeutralMode(NeutralMode::Brake);
    BLSwerveMotor.SetNeutralMode(NeutralMode::Brake);
    BRSwerveMotor.SetNeutralMode(NeutralMode::Brake);

    navX.ZeroYaw();

    FLCANCoder.ConfigAbsoluteSensorRange(Signed_PlusMinus180);
    FRCANCoder.ConfigAbsoluteSensorRange(Signed_PlusMinus180);
    BLCANCoder.ConfigAbsoluteSensorRange(Signed_PlusMinus180);
    BRCANCoder.ConfigAbsoluteSensorRange(Signed_PlusMinus180);

    FLDriveMotor.ConfigPeakOutputForward(mathConst::speedLimit);
    FRDriveMotor.ConfigPeakOutputForward(mathConst::speedLimit);
    BLDriveMotor.ConfigPeakOutputForward(mathConst::speedLimit);
    BRDriveMotor.ConfigPeakOutputForward(mathConst::speedLimit);

    FLDriveMotor.ConfigPeakOutputReverse(-mathConst::speedLimit);
    FRDriveMotor.ConfigPeakOutputReverse(-mathConst::speedLimit);
    BLDriveMotor.ConfigPeakOutputReverse(-mathConst::speedLimit);
    BRDriveMotor.ConfigPeakOutputReverse(-mathConst::speedLimit);
    
    FLCANCoder.ConfigMagnetOffset(CANCoderOffsets::kFrontLeft);
    FRCANCoder.ConfigMagnetOffset(CANCoderOffsets::kFrontRight);
    BLCANCoder.ConfigMagnetOffset(CANCoderOffsets::kBackLeft);
    BRCANCoder.ConfigMagnetOffset(CANCoderOffsets::kBackRight);
}
void Robot::RobotPeriodic() 
{
    // Get the rotation of the robot from the gyro.
    frc::Rotation2d rotation = navX.GetRotation2d();

    // Update the pose.
    auto pose = odometry.Update(
        rotation,
        {
        frc::SwerveModulePosition{TalonFXToInches(FLDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FLCANCoder.GetPosition()})}, 
        frc::SwerveModulePosition{TalonFXToInches(FRDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FRCANCoder.GetPosition()})}, 
        frc::SwerveModulePosition{TalonFXToInches(BLDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BLCANCoder.GetPosition()})}, 
        frc::SwerveModulePosition{TalonFXToInches(BRDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BRCANCoder.GetPosition()})}
        });

    // Display pose and rotation on SmartDashboard.
    frc::SmartDashboard::PutNumber("X ", pose.X().value());
    frc::SmartDashboard::PutNumber("Y ", pose.Y().value());
    frc::SmartDashboard::PutNumber("theta ", rotation.Degrees().value());

    table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    tx = table -> GetNumber("tx", 0.0);
    ty = table -> GetNumber("ty", 0.0);
    table -> PutNumber("pipeline", pipeline+1);
    angleToGoalDegrees = Limelight::limelightMountAngleDegrees + ty;
    distanceFromLimelightToGoalInches = (Limelight::goalHeightInches - Limelight::limelightLensHeightInches)/tan(getRadian(angleToGoalDegrees));
}

void Robot::AutonomousInit() 
{
}
void Robot::AutonomousPeriodic() 
{
    
    if(goBalanceDog){
        double balanceVelocity = getAutoBalanceVelocity(navX.GetRoll());
        frc::SmartDashboard::PutNumber("Yaw", navX.GetAngle());
        frc::SmartDashboard::PutNumber("Roll", navX.GetRoll());
        frc::SmartDashboard::PutNumber("Pitch", navX.GetPitch());
        frc::SmartDashboard::PutNumber("balanceVelocity ", balanceVelocity); //debug
        FLSwerveMotor.Set(TalonFXControlMode::PercentOutput, deadband(swerveCalcs(FLCANCoder.GetAbsolutePosition(), 0), 2));
        FRSwerveMotor.Set(TalonFXControlMode::PercentOutput, deadband(swerveCalcs(FRCANCoder.GetAbsolutePosition(), 0), 2));
        BLSwerveMotor.Set(TalonFXControlMode::PercentOutput, deadband(swerveCalcs(BLCANCoder.GetAbsolutePosition(), 0), 2));
        BRSwerveMotor.Set(TalonFXControlMode::PercentOutput, deadband(swerveCalcs(BRCANCoder.GetAbsolutePosition(), 0), 2));
        FLDriveMotor.Set(TalonFXControlMode::PercentOutput, balanceVelocity);
        FRDriveMotor.Set(TalonFXControlMode::PercentOutput, balanceVelocity);
        BLDriveMotor.Set(TalonFXControlMode::PercentOutput, balanceVelocity);
        BRDriveMotor.Set(TalonFXControlMode::PercentOutput, balanceVelocity);
    }
    else if(fabs(navX.GetRoll())>(11)){
        goBalanceDog = true;
    }
    else {
        FLSwerveMotor.Set(TalonFXControlMode::PercentOutput, deadband(swerveCalcs(FLCANCoder.GetAbsolutePosition(), 0), 2));
        FRSwerveMotor.Set(TalonFXControlMode::PercentOutput, deadband(swerveCalcs(FRCANCoder.GetAbsolutePosition(), 0), 2));
        BLSwerveMotor.Set(TalonFXControlMode::PercentOutput, deadband(swerveCalcs(BLCANCoder.GetAbsolutePosition(), 0), 2));
        BRSwerveMotor.Set(TalonFXControlMode::PercentOutput, deadband(swerveCalcs(BRCANCoder.GetAbsolutePosition(), 0), 2));
        FLDriveMotor.Set(TalonFXControlMode::PercentOutput, 0.10);
        FRDriveMotor.Set(TalonFXControlMode::PercentOutput, 0.10);
        BLDriveMotor.Set(TalonFXControlMode::PercentOutput, 0.10);
        BRDriveMotor.Set(TalonFXControlMode::PercentOutput, 0.10);
    }
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic()
{
    // moduleDesiredStates contains all calculated desired values.
    // .flm is "Front left magnitude" (percentage)
    // .fla is "Front left angle" (degrees)
    // fl[], fr[], bl[], br[]

    if (controller.GetAButton()) {
        pipeline = 1;
        LimelightDifference = deadband(distanceFromLimelightToGoalInches-45, 1); // 50 = desired
        if (abs(LimelightDifference)>20){
            LimelightDifference = -(std::signbit(tx)-0.5)*2;
        }
        else {
            LimelightDifference = LimelightDifference/20;
        }
        LimelightSlew = demonicslew(LimelightSlew, LimelightDifference);
        frc::SmartDashboard::PutNumber("LimelightDifference", LimelightSlew);
        moduleDesiredStates = swerveKinematics(0, LimelightSlew, tx/28, 180);
    }
    else if (controller.GetBButton())
    {
        pipeline = 2;
        LimelightDifference = deadband(distanceFromLimelightToGoalInches-45, 1); // 50 = desired
        if (abs(LimelightDifference)>20){
            LimelightDifference = -(std::signbit(tx)-0.5)*2;
        }
        else {
            LimelightDifference = LimelightDifference/20;
        }
        LimelightSlew = demonicslew(LimelightSlew, LimelightDifference);
        frc::SmartDashboard::PutNumber("LimelightDifference", LimelightSlew);
        moduleDesiredStates = swerveKinematics(0, LimelightSlew, tx/28, 180);
    }
    else {
        moduleDesiredStates = swerveKinematics(deadband(controller.GetLeftX(), 0), deadband(controller.GetLeftY(), 0), deadband(controller.GetRightX(), 0), navX.GetAngle());
    }
    frc::SmartDashboard::PutNumber("Dist", distanceFromLimelightToGoalInches);
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

    setDesiredState(FLSwerveMotor, FLDriveMotor, &FLSwerveState, FLCANCoder.GetAbsolutePosition(), desiredTurnFL, &FLDriveState, moduleDesiredStates.flm, moduleDesiredStates.fla);
    setDesiredState(FRSwerveMotor, FRDriveMotor, &FRSwerveState, FRCANCoder.GetAbsolutePosition(), desiredTurnFR, &FRDriveState, moduleDesiredStates.frm, moduleDesiredStates.fra);
    setDesiredState(BLSwerveMotor, BLDriveMotor, &BLSwerveState, BLCANCoder.GetAbsolutePosition(), desiredTurnBL, &BLDriveState, moduleDesiredStates.blm, moduleDesiredStates.bla);
    setDesiredState(BRSwerveMotor, BRDriveMotor, &BRSwerveState, BRCANCoder.GetAbsolutePosition(), desiredTurnBR, &BRDriveState, moduleDesiredStates.brm, moduleDesiredStates.bra);
    
// Debug Math Outputs
    // Drive motor speeds (percentage)
    logSwerveNumber("Magnitude", FLDriveState, FRDriveState, BLDriveState, BRDriveState);
    logSwerveNumber("drive", moduleDesiredStates.flm,moduleDesiredStates.frm,moduleDesiredStates.blm,moduleDesiredStates.brm);
    // Desired turn angles (degrees)
    logSwerveNumber("Desired Angle", desiredTurnFL, desiredTurnFR, desiredTurnBL, desiredTurnBR);

    // CANCoder Absolute Readings
    logSwerveNumber("CANCoder", FLCANCoder.GetAbsolutePosition(), FRCANCoder.GetAbsolutePosition(), BLCANCoder.GetAbsolutePosition(), BRCANCoder.GetAbsolutePosition());

    // Gyro angle (degrees)
    frc::SmartDashboard::PutNumber("Yaw", navX.GetAngle());
    frc::SmartDashboard::PutNumber("Roll", navX.GetRoll());
    frc::SmartDashboard::PutNumber("Pitch", navX.GetPitch());
    frc::SmartDashboard::PutNumber("Rate", navX.GetRate());

// --------- this section is for testing; kenta chooses which features stay and the trigger things are only for configuring preference
    // Zero gyro (press d-pad in whatever direction the PDP is relative to the North you want)
    if (controller.GetPOV()!=-1)
    {
        navX.ZeroYaw();
        navX.SetAngleAdjustment(controller.GetPOV());
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