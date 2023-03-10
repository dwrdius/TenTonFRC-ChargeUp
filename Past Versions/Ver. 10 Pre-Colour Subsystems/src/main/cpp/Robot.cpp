#include "Robot.h"

// Initialize devices.

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
TalonFX ArmMotor{CanIDs::kArmMotor};
TalonFX ShooterTop{CanIDs::kShooterTop};
TalonFX ShooterBottom{CanIDs::kShooterBottom};

// Encoders
CANCoder FLCANCoder{CanIDs::kFLCANCoder};
CANCoder FRCANCoder{CanIDs::kFRCANCoder};
CANCoder BLCANCoder{CanIDs::kBLCANCoder};
CANCoder BRCANCoder{CanIDs::kBRCANCoder};


//Intake ---------------------------------------------------------------------------------------

// Intake Neo Motors 
rev::CANSparkMax IntakeMaster{RevIDs::kIntakeMaster, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
rev::CANSparkMax IntakeSlave{RevIDs::kIntakeSlave, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

// Intake Falcons
TalonFX IntakeUpDown {CanIDs::kIntakeUpDown};

// ----------------------------------------------------------------------------------------------

// Gyro
AHRS navX{frc::SPI::kMXP};

// LED
frc::PWMSparkMax LED{RevIDs::kLED};

// wall of not constant variable shame 
// Limelight shenanigans
auto table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
double tx;
double ty;
std::vector<double> aprilPos;

int pipeline = 1;

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

// auto
double diffX;
double diffY;
double diffAngle;
double linearDisplacement;
double translationAngle;
int autoIndex = 0;

bool goBalanceDog = true;
bool LEDStrobe = true;

// true = down + can intake; false = up
bool intakeBoolState = true;
double intakeDefaultState;

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

frc::Pose2d currentPose{0_m, 0_m, 0_rad};

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
desiredSwerveModule autoDesiredStates = swerveKinematics(0, 0, 0, 0);

frc::SwerveDriveKinematics<4> kinematics{
  Odometry::kFLLocation, 
  Odometry::kFRLocation,
  Odometry::kBLLocation, 
  Odometry::kBRLocation
};

// Convert SelectedSensorPosition (TalonFX Units) to Inches.
units::length::inch_t TalonFXToInches(double selectedSensorPosition) {
  return units::length::inch_t{selectedSensorPosition / 2048.0 * 4 * M_PI / 6.75};
}

// Convert SelectedSensorPosition (TalonFX Units) to Degrees.
units::angle::degree_t TalonFXToDegrees(double selectedSensorPosition) {
  return units::angle::degree_t{(selectedSensorPosition * (360.0/2048.0))};
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
  frc::Pose2d{0_in, 0_in, 0_deg}
};

// array[index][NESW x, NESW y, NESW angle]
void moveToCoord(double autoCommandList[][3])
{ 
    diffX = autoCommandList[autoIndex][0] + currentPose.Y().value();
    diffY = autoCommandList[autoIndex][1] - currentPose.X().value();
    diffAngle = fmod(autoCommandList[autoIndex][2] - navX.GetYaw(), 360.0);
    translationAngle = atan2(diffX, diffY);

    if (diffAngle > 180.0)
    {
        diffAngle = diffAngle - 360.0;
    }
    else if (diffAngle < -180.0)
    {
        diffAngle = diffAngle + 360.0;
    }

    linearDisplacement = magnitude(diffX, diffY);
    autoRotationScalarFromCoords(diffAngle, linearDisplacement);

    linearDisplacement = deadband(linearDisplacement, 1) / 48;  // 48 inches is the Kp for now 
                                                                //* using limelight distance allowance for this too
    if (!linearDisplacement)
    {
        autoDesiredStates = swerveKinematics(0, 0, diffAngle/25, 0); // 25 is an arbitrary Kp again
        if (!deadband(diffAngle, 2))
        {
            autoIndex++;
        }
    }
    else
    {
        autoDesiredStates = swerveKinematics(linearDisplacement*sin(translationAngle), linearDisplacement*cos(translationAngle), linearDisplacement, navX.GetYaw());
    }
    if (autoDesiredStates.fla < 600.0)
    {
        // Update wheel angles for the turn motors to read
        desiredTurnFL = autoDesiredStates.fla;
        desiredTurnFR = autoDesiredStates.fra;
        desiredTurnBL = autoDesiredStates.bla;
        desiredTurnBR = autoDesiredStates.bra;
    }

    setDesiredState(FLSwerveMotor, FLDriveMotor, &FLSwerveState, FLCANCoder.GetAbsolutePosition(), desiredTurnFL, &FLDriveState, autoDesiredStates.flm, autoDesiredStates.fla);
    setDesiredState(FRSwerveMotor, FRDriveMotor, &FRSwerveState, FRCANCoder.GetAbsolutePosition(), desiredTurnFR, &FRDriveState, autoDesiredStates.frm, autoDesiredStates.fra);
    setDesiredState(BLSwerveMotor, BLDriveMotor, &BLSwerveState, BLCANCoder.GetAbsolutePosition(), desiredTurnBL, &BLDriveState, autoDesiredStates.blm, autoDesiredStates.bla);
    setDesiredState(BRSwerveMotor, BRDriveMotor, &BRSwerveState, BRCANCoder.GetAbsolutePosition(), desiredTurnBR, &BRDriveState, autoDesiredStates.brm, autoDesiredStates.bra);
}

void Robot::RobotInit() 
{
    ArmMotor.ConfigForwardSoftLimitThreshold(90/360*2048 * mathConst::armGearRatio); //experimental will change
    ArmMotor.ConfigForwardSoftLimitEnable(true);
    ArmMotor.ConfigReverseSoftLimitThreshold(0); //dunno reverse is up or down tho moooooo
    ArmMotor.ConfigReverseSoftLimitEnable(true);


    processBaseDimensions(mathConst::xCoords, mathConst::yCoords);

    IntakeMaster.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    IntakeSlave.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    IntakeUpDown.SetNeutralMode(NeutralMode::Brake);

    IntakeSlave.Follow(IntakeMaster, true);

    FLDriveMotor.SetNeutralMode(NeutralMode::Brake);
    FRDriveMotor.SetNeutralMode(NeutralMode::Brake);
    BLDriveMotor.SetNeutralMode(NeutralMode::Brake);
    BRDriveMotor.SetNeutralMode(NeutralMode::Brake);

    FLSwerveMotor.SetNeutralMode(NeutralMode::Brake);
    FRSwerveMotor.SetNeutralMode(NeutralMode::Brake);
    BLSwerveMotor.SetNeutralMode(NeutralMode::Brake);
    BRSwerveMotor.SetNeutralMode(NeutralMode::Brake);

    ArmMotor.SetNeutralMode(NeutralMode::Brake);
    ShooterTop.SetNeutralMode(NeutralMode::Brake);
    ShooterBottom.SetNeutralMode(NeutralMode::Brake);

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

    // Get the rotation of the robot from the gyro.
    frc::Rotation2d rotation = navX.GetRotation2d();

    intakeDefaultState = IntakeUpDown.GetSelectedSensorPosition();

    // Reset the pose.
    odometry.ResetPosition(
        rotation,
        {
        frc::SwerveModulePosition{TalonFXToInches(FLDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FLCANCoder.GetPosition()})}, 
        frc::SwerveModulePosition{TalonFXToInches(FRDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FRCANCoder.GetPosition()})}, 
        frc::SwerveModulePosition{TalonFXToInches(BLDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BLCANCoder.GetPosition()})}, 
        frc::SwerveModulePosition{TalonFXToInches(BRDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BRCANCoder.GetPosition()})}
        },
        frc::Pose2d{0_m, 0_m, 0_deg});
}
void Robot::RobotPeriodic() 
{
}

void Robot::AutonomousInit() 
{
    // Get the rotation of the robot from the gyro.
    frc::Rotation2d rotation = navX.GetRotation2d();

    // Reset the pose.
    odometry.ResetPosition(
        rotation,
        {
        frc::SwerveModulePosition{TalonFXToInches(FLDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FLCANCoder.GetPosition()})}, 
        frc::SwerveModulePosition{TalonFXToInches(FRDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FRCANCoder.GetPosition()})}, 
        frc::SwerveModulePosition{TalonFXToInches(BLDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BLCANCoder.GetPosition()})}, 
        frc::SwerveModulePosition{TalonFXToInches(BRDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BRCANCoder.GetPosition()})}
        },
        frc::Pose2d{0_m, 0_m, 0_deg});
}

void Robot::AutonomousPeriodic() 
{
    // Get the rotation of the robot from the gyro.
    frc::Rotation2d rotation = navX.GetRotation2d();

    // Update the currentPose.
    currentPose = odometry.Update(
        rotation,
        {
        frc::SwerveModulePosition{TalonFXToInches(FLDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FLCANCoder.GetPosition()})}, 
        frc::SwerveModulePosition{TalonFXToInches(FRDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FRCANCoder.GetPosition()})}, 
        frc::SwerveModulePosition{TalonFXToInches(BLDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BLCANCoder.GetPosition()})}, 
        frc::SwerveModulePosition{TalonFXToInches(BRDriveMotor.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BRCANCoder.GetPosition()})}
        });

    // Display currentPose and rotation on SmartDashboard.
    frc::SmartDashboard::PutNumber("X ", currentPose.X().value());
    frc::SmartDashboard::PutNumber("Y ", currentPose.Y().value());



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
        moveToCoord(*autonomous::auto_ptr);
    }
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic()
{
    // moduleDesiredStates contains all calculated desired values.
    // .flm is "Front left magnitude" (percentage)
    // .fla is "Front left angle" (degrees)
    // fl[], fr[], bl[], br[]
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    tx = table -> GetNumber("tx", 0.0);
    ty = table -> GetNumber("ty", 0.0);
    frc::SmartDashboard::PutNumber("tx", tx);
    frc::SmartDashboard::PutNumber("ty", ty);
    try{
        aprilPos = table -> GetNumberArray("botpose_wpiblue", std::vector<double>(6));
        frc::SmartDashboard::PutNumber("april pos", aprilPos.at(0));
        frc::SmartDashboard::PutNumber("april pos1", aprilPos.at(1));
        frc::SmartDashboard::PutNumber("april pos2", aprilPos.at(2));
        frc::SmartDashboard::PutNumber("april pos3", aprilPos.at(3));
        frc::SmartDashboard::PutNumber("april pos4", aprilPos.at(4));
        frc::SmartDashboard::PutNumber("april pos5", aprilPos.at(5));
        throw 505;
    }
    catch (...){

    }
    angleToGoalDegrees = Limelight::limelightMountAngleDegrees + ty;
    distanceFromLimelightToGoalInches = (Limelight::goalHeightInches - Limelight::limelightLensHeightInches)/tan(getRadian(angleToGoalDegrees));

//Retroreflective Tape Tracking
    if (controller.GetAButton()) {
        table -> PutNumber("pipeline", 1);
        LimelightDifference = deadband(distanceFromLimelightToGoalInches-45, 1); // 50 = desired
        
        frc::SmartDashboard::PutNumber("Limelight Distance", LimelightDifference);


        if (abs(LimelightDifference)>20){
            LimelightDifference = -(std::signbit(tx)-0.5)*2;
        }
        else {
            LimelightDifference = LimelightDifference/20;
        }
        LimelightSlew = slew(LimelightSlew, LimelightDifference, 1);
        frc::SmartDashboard::PutNumber("LimelightDifference", LimelightSlew);
        moduleDesiredStates = swerveKinematics(0, LimelightSlew, tx/28, 180);

        frc::SmartDashboard::PutNumber("Limelight", LimelightSlew);
        
        if (!LimelightSlew)
        {
            LED.Set(-0.69);
        }
        else
        {
            LED.Set(0.99);
        }
    }
    //Apriltag tracking
    else if (controller.GetBButton())
    {
        table -> PutNumber("pipeline", 2);
        LimelightDifference = deadband(distanceFromLimelightToGoalInches-45, 1); // 50 = desired
        if (abs(LimelightDifference)>20){
            LimelightDifference = -(std::signbit(tx)-0.5)*2;
        }
        else {
            LimelightDifference = LimelightDifference/20;
        }
        LimelightSlew = slew(LimelightSlew, LimelightDifference, 1);
        frc::SmartDashboard::PutNumber("LimelightDifference", LimelightSlew);
        moduleDesiredStates = swerveKinematics(0, LimelightSlew, tx/28, 180);
        if (!LimelightSlew)
        {
            if (LEDStrobe)
            {
                LED.Set(-0.55);
            }
            else{
                LED.Set(0.99);
            }
            LEDStrobe = !LEDStrobe;
        }
        else
        {
            LED.Set(0.99);
        }
    }
    //Intake's intake mechanism
    
    if(controller.GetYButton()){
        IntakeMaster.Set(1);
    }
    
//Intake up down mechanism
    if(controller.GetXButtonPressed()){
        IntakeUpDown.Set(TalonFXControlMode::Position, intakeDefaultState - intakeBoolState*(2048.0/95.0)*mathConst::intakeGearRatio);
        intakeBoolState = !intakeBoolState;
        
    }
    else {
        moduleDesiredStates = swerveKinematics(deadband(controller.GetLeftX(), 0), deadband(controller.GetLeftY(), 0), deadband(controller.GetRightX(), 0), navX.GetAngle());
    }
    frc::SmartDashboard::PutNumber("Dist", distanceFromLimelightToGoalInches);

//     if (controllerAux.GetXButton()){
//         ShooterTop.Set(ControlMode::PercentOutput, 0.4);
//         ShooterBottom.Set(ControlMode::PercentOutput, -0.4);
//     }
//     if (controllerAux.GetYButton()){
//         ShooterTop.Set(ControlMode::PercentOutput, -0.8);
//         ShooterBottom.Set(ControlMode::PercentOutput, 0.9);
//     }
// // arm lifting with aux controller (manually)
//     ArmMotor.Set(ControlMode::PercentOutput, controllerAux.GetLeftY());
//     frc::SmartDashboard::PutNumber("armPos", ArmMotor.GetSelectedSensorPosition()*(360.0/2048.0));

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