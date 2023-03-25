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

        // // Colour Sensor
        // rev::ColorSensorV3 m_colorSensor{frc::I2C::Port::kOnboard};

// Limit switches
frc::DigitalInput armLimitSwitch{0};
frc::DigitalInput intakeLimitSwitch{1};

double ArmOutCurr = 0;
double IntakeOutCurr = 0;
double NeoOutCurr = 0;

int autostop;
int heresyTimer = 0;

// true = going down; false = going up
bool intakeState = true;
double intakePos; 
double intakePercentage;
int intakeStage;
bool intakeActive = false;

// true = auto; false = manual
int armState = 0;
double armPos;
double armPercentage;


// wall of not constant variable shame 
// Limelight shenanigans
auto table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

double ty;

double LLAlignTranslation;

double angleToGoalDegrees;
double distanceFromLimelightToGoalInches;

double LimelightDifference=1;
double LimelightSlew;

double heightToGoal;

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

bool cubeAutonomous = true;
bool balance = false;
bool coneCollect = false;
bool onChargingStation = false;


        // // colour
        // rev::ColorMatch m_colorMatcher;
        // frc::Color detectedColor;
        // std::string colorstring;
        // frc::Color matchedColor;
        // double confidence;

bool throttle;

double alignEast;
double alignSouth;

bool intakeLimitUpdate;

int allianceColor;

auto neo13 = IntakeFollower.GetEncoder();
auto neo14 = IntakeLeader.GetEncoder();

frc::SendableChooser<std::string> m_chooser;
const std::string kCubeConeAlignLeft = "LeftAlign";
const std::string kCubeConeAlignRight = "RightAlign";
const std::string kCubeBalanceLeft = "LeftBalance";
const std::string kCubeBalanceRight = "RightBalance";
const std::string kCubeBalanceCentre = "CentreBalance";
std::string m_autoSelected;


// Stores all Swerve Module desired values
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

frc::Pose2d currentPose{0_in, 0_in, 0_deg};

// Input degrees
// Automatically applies deadbands
// Outputs a swerveModule object 
// (percentage speeds and angles in the order FL, FR, BL, BR)
desiredSwerveModule swerveKinematics(double xLeft, double yLeft, double xRight, double gyro, bool exponentialTrue)
{
    // cmath reads radians; applies deadbands
    gyro = getRadian(gyro);
    
    // Creates a unit vector multiplied by right joystick input and proportionally scaled by "rotationVectorMultiplier"
    double rotationScalar=0;
    if(xRight)
    {
        rotationScalar = pow(xRight, 2)*xRight/abs(xRight);
    }
    double joystickMagnitude=1;
    Point posVector = Point(0.0, 0.0);
    if (ArmMotor.GetSelectedSensorPosition())
    {
        joystickMagnitude = fmin(1, abs(20000/ArmMotor.GetSelectedSensorPosition()));
    }
    double exponentialModification = magnitude(xLeft, yLeft);
    if (exponentialTrue)
    {
        exponentialModification = exponentiate(exponentialModification, 2);
    } 
    joystickMagnitude = joystickMagnitude*exponentialModification;
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
    
    if(!exponentialModification)
    {
        limitingScalar = limitingScalar * 5;
    }

    // Convert to Polar vectors for speed and direction for swerve modules
    Point physFL = Point(magnitudes[0]/limitingScalar, atan2(rawFL.x, rawFL.y));
    Point physFR = Point(magnitudes[1]/limitingScalar, atan2(rawFR.x, rawFR.y));
    Point physBL = Point(magnitudes[2]/limitingScalar, atan2(rawBL.x, rawBL.y));
    Point physBR = Point(magnitudes[3]/limitingScalar, atan2(rawBR.x, rawBR.y));

    return desiredSwerveModule(physFL.x, getDegree(physFL.y), physFR.x, getDegree(physFR.y), physBL.x, getDegree(physBL.y), physBR.x, getDegree(physBR.y));
}

desiredSwerveModule moduleDesiredStates = swerveKinematics(0, 0, 0, 0, false);
desiredSwerveModule autoDesiredStates = swerveKinematics(0, 0, 0, 0, false);

frc::SwerveDriveKinematics<4> kinematics{
  Odometry::kFLLocation, 
  Odometry::kFRLocation,
  Odometry::kBLLocation, 
  Odometry::kBRLocation
};

// Convert SelectedSensorPosition (TalonFX Units) to Inches.
units::length::inch_t TalonFXToInches(double selectedSensorPosition) {
  return units::length::inch_t{selectedSensorPosition / 2048.0 * 4 * M_PI / 6.75 * 1.33};
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
void moveToCoord()
{ 
    diffX = -(autonomous::kAutoCommandList[autoIndex][0] + currentPose.Y().value()*39.37);
    diffY = (autonomous::kAutoCommandList[autoIndex][1] - currentPose.X().value()*39.37);
    diffAngle = fmod(autonomous::kAutoCommandList[autoIndex][2] - navX.GetAngle(), 360.0);
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

    linearDisplacement = deadband(linearDisplacement, 1) / 30;  // 30 inches is the Kp for now 
                                                                //* using limelight distance allowance for this too
    if (!linearDisplacement)
    {
        autoDesiredStates = swerveKinematics(0, 0, diffAngle/5, 0, false);
        if (!deadband(diffAngle, 3))
        {
            autoIndex++;
        }
    }
    else
    {
        autoDesiredStates = swerveKinematics(linearDisplacement*sin(translationAngle), linearDisplacement*cos(translationAngle), linearDisplacement, navX.GetAngle(), false);
    }
    frc::SmartDashboard::PutNumber("Angle", navX.GetAngle());
    // frc::SmartDashboard::PutNumber("auto y", linearDisplacement*cos(translationAngle));
    // frc::SmartDashboard::PutNumber("auto xRight", linearDisplacement);
    // frc::SmartDashboard::PutNumber("auto rotation", mathConst::rotationVectorMultiplier);
}

void moveToPosition(int x, int y, int theta)
{ 
    diffX = -(x + currentPose.Y().value()*39.37);
    diffY = (y - currentPose.X().value()*39.37);
    diffAngle = fmod(theta - navX.GetAngle(), 360.0);
    translationAngle = atan2(diffX, diffY);

    diffAngle = constrict180(diffAngle);

    linearDisplacement = magnitude(diffX, diffY);
    autoRotationScalarFromCoords(diffAngle, linearDisplacement);

    linearDisplacement = deadband(linearDisplacement, 1) / 30;  // 30 inches is the Kp for now 
                                                                //* using limelight distance allowance for this too
    if (!linearDisplacement)
    {
        autoDesiredStates = swerveKinematics(0, 0, diffAngle/5, 0, false);
        if (!deadband(diffAngle, 3))
        {
            autoIndex++;
        }
    }
    else
    {
        autoDesiredStates = swerveKinematics(linearDisplacement*sin(translationAngle), linearDisplacement*cos(translationAngle), linearDisplacement, navX.GetAngle(), false);
    }
    frc::SmartDashboard::PutNumber("Angle", navX.GetAngle());
}

void Robot::RobotInit() 
{
    processBaseDimensions(mathConst::xCoords, mathConst::yCoords);

            // m_colorMatcher.AddColorMatch(Colours::KYellowTarget);
            // m_colorMatcher.AddColorMatch(Colours::KPurpleTarget);

    //Soft limits (motor, forward limit, reverse limit, gear ratio)
    // setTalonSoftLimit(ArmMotor, 30, -30, mathConst::armGearRatio);
    ArmMotor.ConfigForwardSoftLimitEnable(false,0);
    ArmMotor.ConfigReverseSoftLimitEnable(false, 0);

    IntakeUpDown.ConfigForwardSoftLimitEnable(false);
    IntakeUpDown.ConfigReverseSoftLimitEnable(false);

    //Set Leader Follower motors (previously Master)
    IntakeLeader.RestoreFactoryDefaults();
    IntakeFollower.RestoreFactoryDefaults();
    IntakeLeader.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    IntakeFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    IntakeFollower.Follow(IntakeLeader, true);

    configSwerveModule(FLDriveMotor, FLSwerveMotor, FLCANCoder, CANCoderOffsets::kFrontLeft);
    configSwerveModule(FRDriveMotor, FRSwerveMotor, FRCANCoder, CANCoderOffsets::kFrontRight);
    configSwerveModule(BLDriveMotor, BLSwerveMotor, BLCANCoder, CANCoderOffsets::kBackLeft);
    configSwerveModule(BRDriveMotor, BRSwerveMotor, BRCANCoder, CANCoderOffsets::kBackRight);

    ArmMotor.SetNeutralMode(NeutralMode::Brake);
    ShooterTop.SetNeutralMode(NeutralMode::Brake);
    ShooterBottom.SetNeutralMode(NeutralMode::Brake);
    IntakeUpDown.SetNeutralMode(NeutralMode::Brake);

    navX.ZeroYaw();
    
    limitSpeeds(ArmMotor, 0.3);
    ArmMotor.ConfigPeakOutputReverse(-0.5);
    limitSpeeds(ShooterTop, 1);
    limitSpeeds(ShooterBottom, 1);
    
    IntakeUpDown.ConfigPeakOutputReverse(-0.3);
    IntakeUpDown.ConfigPeakOutputForward(0.15);

    IntakeUpDown.SetSelectedSensorPosition(0);
    ArmMotor.SetSelectedSensorPosition(0);

    m_chooser.SetDefaultOption("kCubeBalanceCentre", kCubeBalanceCentre);
    m_chooser.AddOption("kCubeConeAlignLeft", kCubeConeAlignLeft);
    m_chooser.AddOption("kCubeConeAlignRight", kCubeConeAlignRight);
    m_chooser.AddOption("kCubeBalanceLeft", kCubeBalanceLeft);
    m_chooser.AddOption("kCubeBalanceRight", kCubeBalanceRight);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}
void Robot::RobotPeriodic() 
{
    frc::SmartDashboard::UpdateValues();
    frc::SmartDashboard::PutString("Selected Auto", m_chooser.GetSelected());
    frc::SmartDashboard::PutNumber("Arm Limit", armLimitSwitch.Get());
    frc::SmartDashboard::PutNumber("Intake Limit", intakeLimitSwitch.Get());
    frc::SmartDashboard::PutNumber("Neo14", neo14.GetVelocity());
    frc::SmartDashboard::PutNumber("Neo13", neo13.GetVelocity());

    if(frc::DriverStation::GetAlliance() == frc::DriverStation::kRed)
    {
        allianceColor = Alliances::Red;    
    }
    else if(frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
    {
        allianceColor = Alliances::Blue;
    }

    NeoOutCurr = fmax(NeoOutCurr, IntakeLeader.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("neo current", NeoOutCurr);
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
        frc::Pose2d{0_in, 0_in, 0_deg});
    
    IntakeUpDown.SetNeutralMode(NeutralMode::Brake);
    navX.ZeroYaw();
    navX.SetAngleAdjustment(0);

    limitSpeeds(FLDriveMotor, 0.4);
    limitSpeeds(FLSwerveMotor, 0.4);
    limitSpeeds(FRDriveMotor, 0.4);
    limitSpeeds(FRSwerveMotor, 0.4);
    limitSpeeds(BLDriveMotor, 0.4);
    limitSpeeds(BLSwerveMotor, 0.4);
    limitSpeeds(BRDriveMotor, 0.4);
    limitSpeeds(BRSwerveMotor, 0.4);
    mathConst::pseudoSpeedLimit = 0.4;

    autostop = 0;
}

/*
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
    frc::SmartDashboard::PutNumber("X ", -currentPose.Y().value()*39.37);
    frc::SmartDashboard::PutNumber("Y ", currentPose.X().value()*39.37);
    frc::SmartDashboard::PutNumber("intake limitswitch", intakeLimitSwitch.Get());
    frc::SmartDashboard::PutNumber("autoindex", autoIndex);
    if(coneCollect){
        if (autoIndex == 0 || autoIndex == 1 || autoIndex == 5 || autoIndex == 6)
        {
            moveToCoord();
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
        else if (autoIndex == 2)
        {
            table -> PutNumber("pipeline", Limelight::Cone);
            ty = table -> GetNumber("ty", 0.0);
            if (deadband(ty, 5) < 0)
            {
                LLAlignTranslation = -deadband(ty-0.5, 5)/40;
            }
            else if (deadband(ty, 5))
            {
                LLAlignTranslation = -deadband(ty+0.5, 5)/40;
            }
            else
            {
                LLAlignTranslation = 0;
            }
            autoDesiredStates = swerveKinematics(LLAlignTranslation, 0, 0, navX.GetAngle(), false);
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
            
            intakePos = 30000;
            intakePercentage = (intakePos-IntakeUpDown.GetSelectedSensorPosition())/80000;
            if (intakePercentage < 0 && !intakeLimitSwitch.Get())
            {
                intakePercentage = 0;
            }
            frc::SmartDashboard::PutNumber("Intake Error", intakePos-IntakeUpDown.GetSelectedSensorPosition());
            if (intakePos - IntakeUpDown.GetSelectedSensorPosition() < 1500 && !deadband(ty, 3))
            {
                intakePercentage = 0;
                autoIndex = 3;
            }
            IntakeUpDown.Set(TalonFXControlMode::PercentOutput, intakePercentage);
        }
        else if (autoIndex == 3){
            //cry
            
            if(heresyTimer > 40)
            {
                FLSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                FRSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                BLSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                BRSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                FLDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                FRDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                BLDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                BRDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                
                IntakeLeader.Set(0);

                autoIndex = 4;
            }
            else
            {
                heresyTimer++;
                table -> PutNumber("pipeline", Limelight::Cone);
                ty = table -> GetNumber("ty", 0.0);
                if (deadband(ty, 5) < 0)
                {
                    LLAlignTranslation = -deadband(ty-0.5, 5)/50;
                }
                else if (deadband(ty, 5))
                {
                    LLAlignTranslation = -deadband(ty+0.5, 5)/50;
                }
                else
                {
                    LLAlignTranslation = 0;
                }

                autoDesiredStates = swerveKinematics(LLAlignTranslation, 0.5, 0, navX.GetAngle(), false); // fmod(180-navX.GetAngle(), 360.0)/25
                IntakeLeader.Set(0.6);
                
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
        }
        else if (autoIndex == 4)
        {
            intakePos = 4500;
            intakePercentage = (intakePos-IntakeUpDown.GetSelectedSensorPosition())/50000;
            if (intakePos-IntakeUpDown.GetSelectedSensorPosition() < 0)
            {
                intakePercentage = fmin(-0.1, intakePercentage);
            }
            else
            {
                intakePercentage = fmax(0.1, intakePercentage);
            }
            if (intakePercentage < 0 && !intakeLimitSwitch.Get())
            {
                intakePercentage = 0;
            }
            frc::SmartDashboard::PutNumber("Intake Error", intakePos-IntakeUpDown.GetSelectedSensorPosition());
            if (abs(intakePos - IntakeUpDown.GetSelectedSensorPosition()) < 500)
            {
                autoIndex = 5;
                intakePercentage = 0;
                IntakeUpDown.Set(TalonFXControlMode::PercentOutput, 0);
                heresyTimer = 0;
            }
            IntakeUpDown.Set(TalonFXControlMode::PercentOutput, intakePercentage);
        }
        else if (autoIndex == 7)
        {
            ty = table -> GetNumber("ty", 0.0);
            double tx = table -> GetNumber("tx", 0.0);
            
            if (deadband(ty, 5) < 0)
            {
                LLAlignTranslation = deadband(ty-0.5, 5)/20;
            }
            else if (deadband(ty, 5))
            {
                LLAlignTranslation = deadband(ty+0.5, 5)/20;
            }
            else
            {
                LLAlignTranslation = 0;
            }
            if (LLAlignTranslation)
            {
                autoDesiredStates = swerveKinematics(LLAlignTranslation, -0.1, 0, navX.GetAngle(), false);
            }
            else if (deadband((tx-21.9), 5))
            {
                autoDesiredStates = swerveKinematics(0, -0.3, 0, navX.GetAngle(), false);
            }
            else
            {
                autoDesiredStates = swerveKinematics(0, 0, 0, 0, false);
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

            armPos = -77000;
            armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
            if (!armLimitSwitch.Get())
            {
                armPercentage = 0;
            }
            ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
            if (!deadband(armPercentage, 4) && !deadband(ty*2, 5) && !deadband((tx-21.9), 5))
            {
                autoIndex = 8;
                heresyTimer = 0;
                ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
                FLSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                FRSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                BLSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                BRSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                FLDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                FRDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                BLDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                BRDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);    
            }
        }
        else if (autoIndex == 8)
        {
            heresyTimer++;
            if (heresyTimer>10)
            {
                ShooterTop.Set(ControlMode::PercentOutput, 0.9);
                ShooterBottom.Set(ControlMode::PercentOutput, 1);
            }
            else{
                ShooterTop.Set(ControlMode::PercentOutput, -0.2);
                ShooterBottom.Set(ControlMode::PercentOutput, -0.2);
            }
        }
        else
        {
            FLSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
            FRSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
            BLSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
            BRSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
            FLDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
            FRDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
            BLDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
            BRDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
            ShooterTop.Set(ControlMode::PercentOutput, 0);
            ShooterBottom.Set(ControlMode::PercentOutput, 0);
            IntakeLeader.Set(0);
            IntakeUpDown.Set(ControlMode::PercentOutput, 0);
            ArmMotor.Set(ControlMode::PercentOutput, 0);
        }
        if (!autoIndex)
        {
            armPos = -4000;
            armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
            if (!armLimitSwitch.Get())
            {
                armPercentage = 0;
            }
            ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
            if(!ArmMotor.GetSelectedSensorVelocity() && !deadband(armPos-ArmMotor.GetSelectedSensorPosition(), 4))
            {
                ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
            }
        }
        else if (autoIndex == 1)
        {
            armPos = -20000;
            armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
            if (!armLimitSwitch.Get())
            {
                armPercentage = 0;
            }
            ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
            if(!ArmMotor.GetSelectedSensorVelocity() && !deadband(armPos-ArmMotor.GetSelectedSensorPosition(), 4))
            {
                ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
            }
        }
        else if (autoIndex == 5)
        {
            IntakeUpDown.Set(TalonFXControlMode::PercentOutput, 0);
            ShooterTop.Set(ControlMode::PercentOutput, -0.4);
            ShooterBottom.Set(ControlMode::PercentOutput, -0.4);
            armPos = -4000;
            armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
            if (!armLimitSwitch.Get())
            {
                armPercentage = 0;
            }
            ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
            if(!deadband(armPos-ArmMotor.GetSelectedSensorPosition(), 4))
            {
                ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
                heresyTimer++;
                if (heresyTimer < 30)
                {
                    IntakeLeader.Set(-0.2);
                }
                else
                {
                    ShooterTop.Set(ControlMode::PercentOutput, 0);
                    ShooterBottom.Set(ControlMode::PercentOutput, 0);
                    IntakeLeader.Set(0);
                }
            }
        }
        else if (autoIndex == 6)
        {
            armPos = -77000;
            armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
            if (!armLimitSwitch.Get())
            {
                armPercentage = 0;
            }
            ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
            heresyTimer = 0;
            table -> PutNumber("pipeline", Limelight::TopReflective);
        }
        else
        {
            ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
        }
    }
    else if (cubeAutonomous) {
        autostop++;
        frc::SmartDashboard::PutNumber("Autostop", autostop);
        if (autostop < 4000)
        {
            armPos = -50000;
            armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
            if (!armLimitSwitch.Get())
            {
                armPercentage = 0;
            }
            ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
            if(ArmMotor.GetSelectedSensorPosition()< -10000)
            {
                autostop = 4000; // arbitrary impossible value
            }
        }
        else
        {
            if (autostop < 8000) // down
            {
                armPos = -50000;
                armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
                if (!armLimitSwitch.Get())
                {
                    armPercentage = 0;
                }
                ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
                
                intakePos = 10000;
                intakePercentage = (intakePos-IntakeUpDown.GetSelectedSensorPosition())/50000;
                frc::SmartDashboard::PutNumber("Intake Error", intakePos-IntakeUpDown.GetSelectedSensorPosition());
                if (intakePos-IntakeUpDown.GetSelectedSensorPosition() < 0)
                {
                    intakePercentage = fmin(-0.1, intakePercentage);
                }
                else
                {
                    intakePercentage = fmax(0.1, intakePercentage);
                }
                if (abs(intakePos - IntakeUpDown.GetSelectedSensorPosition()) < 500 && autostop < 5000 && !deadband(armPos-ArmMotor.GetSelectedSensorPosition(), 4))
                {
                    autostop = 5000;
                    ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
                }
                if (intakePercentage < 0 && !intakeLimitSwitch.Get())
                {
                    intakePercentage = 0;
                }
                IntakeUpDown.Set(TalonFXControlMode::PercentOutput, intakePercentage);
                if(autostop > 5020)
                {
                    IntakeLeader.Set(-1);
                    if (autostop > 5040)
                    {
                        IntakeLeader.Set(0);
                        autostop = 8000;
                    }
                }
            }
            else 
            {
                intakePos = 1000;
                intakePercentage = (intakePos-IntakeUpDown.GetSelectedSensorPosition())/40000;
                frc::SmartDashboard::PutNumber("Intake Error", intakePos-IntakeUpDown.GetSelectedSensorPosition());
                intakePercentage = fmin(-0.1, intakePercentage);
                
                if (abs(intakePos - IntakeUpDown.GetSelectedSensorPosition()) < 1500)
                {
                    autostop = 12000;
                }

                if (intakePercentage < 0 && !intakeLimitSwitch.Get())
                {
                    intakePercentage = 0;
                }
                IntakeUpDown.Set(TalonFXControlMode::PercentOutput, intakePercentage);
                if (autostop > 11999)
                {
                    IntakeUpDown.Set(TalonFXControlMode::PercentOutput, 0);
                    coneCollect = true;
                    cubeAutonomous = false;
                }
            }
        }
    }
}
*/

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
    frc::SmartDashboard::PutNumber("X ", -currentPose.Y().value()*39.37);
    frc::SmartDashboard::PutNumber("Y ", currentPose.X().value()*39.37);
    
    frc::SmartDashboard::PutNumber("intake limitswitch", intakeLimitSwitch.Get());
    frc::SmartDashboard::PutNumber("autoindex", autoIndex);

    if (m_chooser.GetSelected() == kCubeConeAlignLeft || m_chooser.GetSelected() == kCubeConeAlignRight) {
        if(coneCollect){
            if (autoIndex == 0 || autoIndex == 1 || autoIndex == 5)
            {
                moveToCoord();
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
            else if (autoIndex == 2)
            {
                table -> PutNumber("pipeline", Limelight::Cone);
                ty = table -> GetNumber("ty", 0.0);
                if (deadband(ty, 5) < 0)
                {
                    LLAlignTranslation = -deadband(ty-0.5, 5)/40;
                }
                else if (deadband(ty, 5))
                {
                    LLAlignTranslation = -deadband(ty+0.5, 5)/40;
                }
                else
                {
                    LLAlignTranslation = 0;
                }
                autoDesiredStates = swerveKinematics(LLAlignTranslation, 0, 0, navX.GetAngle(), false);
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
                
                intakePos = 30000;
                intakePercentage = (intakePos-IntakeUpDown.GetSelectedSensorPosition())/80000;
                if (intakePercentage < 0 && !intakeLimitSwitch.Get())
                {
                    intakePercentage = 0;
                }
                frc::SmartDashboard::PutNumber("Intake Error", intakePos-IntakeUpDown.GetSelectedSensorPosition());
                if (intakePos - IntakeUpDown.GetSelectedSensorPosition() < 1500 && !deadband(ty, 3))
                {
                    intakePercentage = 0;
                    autoIndex = 3;
                }
                IntakeUpDown.Set(TalonFXControlMode::PercentOutput, intakePercentage);
            }
            else if (autoIndex == 3){
                //cry
                
                if(heresyTimer > 40)
                {
                    FLSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    FRSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    BLSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    BRSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    FLDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    FRDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    BLDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    BRDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    
                    IntakeLeader.Set(0);

                    autoIndex = 4;
                }
                else
                {
                    heresyTimer++;
                    table -> PutNumber("pipeline", Limelight::Cone);
                    ty = table -> GetNumber("ty", 0.0);
                    if (deadband(ty, 5) < 0)
                    {
                        LLAlignTranslation = -deadband(ty-0.5, 5)/50;
                    }
                    else if (deadband(ty, 5))
                    {
                        LLAlignTranslation = -deadband(ty+0.5, 5)/50;
                    }
                    else
                    {
                        LLAlignTranslation = 0;
                    }

                    autoDesiredStates = swerveKinematics(LLAlignTranslation, 0.5, 0, navX.GetAngle(), false); // fmod(180-navX.GetAngle(), 360.0)/25
                    IntakeLeader.Set(0.6);
                    
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
            }
            else if (autoIndex == 4)
            {
                intakePos = 4500;
                intakePercentage = (intakePos-IntakeUpDown.GetSelectedSensorPosition())/50000;
                if (intakePos-IntakeUpDown.GetSelectedSensorPosition() < 0)
                {
                    intakePercentage = fmin(-0.1, intakePercentage);
                }
                else
                {
                    intakePercentage = fmax(0.1, intakePercentage);
                }
                if (intakePercentage < 0 && !intakeLimitSwitch.Get())
                {
                    intakePercentage = 0;
                }
                frc::SmartDashboard::PutNumber("Intake Error", intakePos-IntakeUpDown.GetSelectedSensorPosition());
                if (abs(intakePos - IntakeUpDown.GetSelectedSensorPosition()) < 500)
                {
                    autoIndex = 5;
                    intakePercentage = 0;
                    IntakeUpDown.Set(TalonFXControlMode::PercentOutput, 0);
                    heresyTimer = 0;
                }
                IntakeUpDown.Set(TalonFXControlMode::PercentOutput, intakePercentage);
            }
            else if (autoIndex == 6)
            {
                if (m_chooser.GetSelected() == kCubeConeAlignLeft)
                {
                    moveToPosition(43, 0, 0);
                }
                else
                {
                    moveToPosition(-43, 0, 0);
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
            else if (autoIndex == 7)
            {
                ty = table -> GetNumber("ty", 0.0);
                double tx = table -> GetNumber("tx", 0.0);
                
                if (deadband(ty, 5) < 0)
                {
                    LLAlignTranslation = deadband(ty-0.5, 5)/20;
                }
                else if (deadband(ty, 5))
                {
                    LLAlignTranslation = deadband(ty+0.5, 5)/20;
                }
                else
                {
                    LLAlignTranslation = 0;
                }
                if (LLAlignTranslation)
                {
                    autoDesiredStates = swerveKinematics(LLAlignTranslation, -0.1, 0, navX.GetAngle(), false);
                }
                else if (deadband((tx-21.9), 5))
                {
                    autoDesiredStates = swerveKinematics(0, -0.3, 0, navX.GetAngle(), false);
                }
                else
                {
                    autoDesiredStates = swerveKinematics(0, 0, 0, 0, false);
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

                armPos = -77000;
                armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
                if (!armLimitSwitch.Get())
                {
                    armPercentage = 0;
                }
                ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
                if (!deadband(armPercentage, 4) && !deadband(ty*2, 5) && !deadband((tx-21.9), 5))
                {
                    autoIndex = 8;
                    heresyTimer = 0;
                    ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    FLSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    FRSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    BLSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    BRSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    FLDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    FRDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    BLDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    BRDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);    
                }
            }
            else if (autoIndex == 8)
            {
                heresyTimer++;
                if (heresyTimer>10)
                {
                    ShooterTop.Set(ControlMode::PercentOutput, 0.9);
                    ShooterBottom.Set(ControlMode::PercentOutput, 1);
                }
                else{
                    ShooterTop.Set(ControlMode::PercentOutput, -0.2);
                    ShooterBottom.Set(ControlMode::PercentOutput, -0.2);
                }
            }
            else
            {
                FLSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                FRSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                BLSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                BRSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                FLDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                FRDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                BLDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                BRDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                ShooterTop.Set(ControlMode::PercentOutput, 0);
                ShooterBottom.Set(ControlMode::PercentOutput, 0);
                IntakeLeader.Set(0);
                IntakeUpDown.Set(ControlMode::PercentOutput, 0);
                ArmMotor.Set(ControlMode::PercentOutput, 0);
            }
            if (!autoIndex)
            {
                armPos = -4000;
                armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
                if (!armLimitSwitch.Get())
                {
                    armPercentage = 0;
                }
                ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
                if(!ArmMotor.GetSelectedSensorVelocity() && !deadband(armPos-ArmMotor.GetSelectedSensorPosition(), 4))
                {
                    ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
                }
            }
            else if (autoIndex == 1)
            {
                armPos = -20000;
                armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
                if (!armLimitSwitch.Get())
                {
                    armPercentage = 0;
                }
                ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
                if(!ArmMotor.GetSelectedSensorVelocity() && !deadband(armPos-ArmMotor.GetSelectedSensorPosition(), 4))
                {
                    ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
                }
            }
            else if (autoIndex == 5)
            {
                IntakeUpDown.Set(TalonFXControlMode::PercentOutput, 0);
                ShooterTop.Set(ControlMode::PercentOutput, -0.4);
                ShooterBottom.Set(ControlMode::PercentOutput, -0.4);
                armPos = -4000;
                armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
                if (!armLimitSwitch.Get())
                {
                    armPercentage = 0;
                }
                ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
                if(!deadband(armPos-ArmMotor.GetSelectedSensorPosition(), 4))
                {
                    ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    heresyTimer++;
                    if (heresyTimer < 30)
                    {
                        IntakeLeader.Set(-0.2);
                    }
                    else
                    {
                        ShooterTop.Set(ControlMode::PercentOutput, 0);
                        ShooterBottom.Set(ControlMode::PercentOutput, 0);
                        IntakeLeader.Set(0);
                    }
                }
            }
            else if (autoIndex == 6)
            {
                armPos = -77000;
                armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
                if (!armLimitSwitch.Get())
                {
                    armPercentage = 0;
                }
                ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
                heresyTimer = 0;
                table -> PutNumber("pipeline", Limelight::TopReflective);
            }
            else
            {
                ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
            }
        }
        else if (cubeAutonomous) {
            autostop++;
            frc::SmartDashboard::PutNumber("Autostop", autostop);
            if (autostop < 4000)
            {
                armPos = -50000;
                armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
                if (!armLimitSwitch.Get())
                {
                    armPercentage = 0;
                }
                ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
                if(ArmMotor.GetSelectedSensorPosition()< -10000)
                {
                    autostop = 4000; // arbitrary impossible value
                }
            }
            else
            {
                if (autostop < 8000) // down
                {
                    armPos = -50000;
                    armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
                    if (!armLimitSwitch.Get())
                    {
                        armPercentage = 0;
                    }
                    ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
                    
                    intakePos = 10000;
                    intakePercentage = (intakePos-IntakeUpDown.GetSelectedSensorPosition())/50000;
                    frc::SmartDashboard::PutNumber("Intake Error", intakePos-IntakeUpDown.GetSelectedSensorPosition());
                    if (intakePos-IntakeUpDown.GetSelectedSensorPosition() < 0)
                    {
                        intakePercentage = fmin(-0.1, intakePercentage);
                    }
                    else
                    {
                        intakePercentage = fmax(0.1, intakePercentage);
                    }
                    if (abs(intakePos - IntakeUpDown.GetSelectedSensorPosition()) < 500 && autostop < 5000 && !deadband(armPos-ArmMotor.GetSelectedSensorPosition(), 4))
                    {
                        autostop = 5000;
                        ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    }
                    if (intakePercentage < 0 && !intakeLimitSwitch.Get())
                    {
                        intakePercentage = 0;
                    }
                    IntakeUpDown.Set(TalonFXControlMode::PercentOutput, intakePercentage);
                    if(autostop > 5020)
                    {
                        IntakeLeader.Set(-1);
                        if (autostop > 5040)
                        {
                            IntakeLeader.Set(0);
                            autostop = 8000;
                        }
                    }
                }
                else 
                {
                    intakePos = 1000;
                    intakePercentage = (intakePos-IntakeUpDown.GetSelectedSensorPosition())/40000;
                    frc::SmartDashboard::PutNumber("Intake Error", intakePos-IntakeUpDown.GetSelectedSensorPosition());
                    intakePercentage = fmin(-0.1, intakePercentage);
                    
                    if (abs(intakePos - IntakeUpDown.GetSelectedSensorPosition()) < 1500)
                    {
                        autostop = 12000;
                    }

                    if (intakePercentage < 0 && !intakeLimitSwitch.Get())
                    {
                        intakePercentage = 0;
                    }
                    IntakeUpDown.Set(TalonFXControlMode::PercentOutput, intakePercentage);
                    if (autostop > 11999)
                    {
                        IntakeUpDown.Set(TalonFXControlMode::PercentOutput, 0);
                        coneCollect = true;
                        cubeAutonomous = false;
                    }
                }
            }
        }
    } 
    else if () {
        // empty
    } 
    else if (m_chooser.GetSelected() == kCubeBalanceLeft || m_chooser.GetSelected() == kCubeBalanceRight) {
        if(balance){
            // double roll = deadband(navX.GetRoll(), 3);
            // if (abs(roll)>8)
            // {
            //     onChargingStation = true;
            //     limitSpeeds(FLDriveMotor, 0.3);
            //     limitSpeeds(FLSwerveMotor, 0.3);
            //     limitSpeeds(FRDriveMotor, 0.3);
            //     limitSpeeds(FRSwerveMotor, 0.3);
            //     limitSpeeds(BLDriveMotor, 0.3);
            //     limitSpeeds(BLSwerveMotor, 0.3);
            //     limitSpeeds(BRDriveMotor, 0.3);
            //     limitSpeeds(BRSwerveMotor, 0.3);
            // }
            // if (onChargingStation && abs(roll) < 6 && roll)
            // {
            //     roll = fmax(-roll / 120, -roll/abs(roll)*0.02); // 0.02 is minimum move speed for balance
            // }
            // else{
            //     roll = 1;
            // }

    // ______________________________________________________________________________________________
            
            // double roll = deadband(navX.GetRoll(), 3);
            // if (abs(roll)>8)
            // {
            //     onChargingStation = true;
            //     limitSpeeds(FLDriveMotor, 0.2);
            //     limitSpeeds(FLSwerveMotor, 0.2);
            //     limitSpeeds(FRDriveMotor, 0.2);
            //     limitSpeeds(FRSwerveMotor, 0.2);
            //     limitSpeeds(BLDriveMotor, 0.2);
            //     limitSpeeds(BLSwerveMotor, 0.2);
            //     limitSpeeds(BRDriveMotor, 0.2);
            //     limitSpeeds(BRSwerveMotor, 0.2);
            // }
            // if (onChargingStation)
            // {
            //     roll = -roll / 150;
            // }
            // else{
            //     roll = 1;
            // }
            // autoDesiredStates = swerveKinematics(0, -roll, 0, navX.GetYaw(), false);
            // if (autoDesiredStates.fla < 600.0)
            // {
            //     // Update wheel angles for the turn motors to read
            //     desiredTurnFL = autoDesiredStates.fla;
            //     desiredTurnFR = autoDesiredStates.fra;
            //     desiredTurnBL = autoDesiredStates.bla;
            //     desiredTurnBR = autoDesiredStates.bra;
            // }

            // setDesiredState(FLSwerveMotor, FLDriveMotor, &FLSwerveState, FLCANCoder.GetAbsolutePosition(), desiredTurnFL, &FLDriveState, autoDesiredStates.flm, autoDesiredStates.fla);
            // setDesiredState(FRSwerveMotor, FRDriveMotor, &FRSwerveState, FRCANCoder.GetAbsolutePosition(), desiredTurnFR, &FRDriveState, autoDesiredStates.frm, autoDesiredStates.fra);
            // setDesiredState(BLSwerveMotor, BLDriveMotor, &BLSwerveState, BLCANCoder.GetAbsolutePosition(), desiredTurnBL, &BLDriveState, autoDesiredStates.blm, autoDesiredStates.bla);
            // setDesiredState(BRSwerveMotor, BRDriveMotor, &BRSwerveState, BRCANCoder.GetAbsolutePosition(), desiredTurnBR, &BRDriveState, autoDesiredStates.brm, autoDesiredStates.bra);
        }
        else if(coneCollect){
            if (autoIndex == 0 || autoIndex == 1)
            {
                moveToCoord();
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
            else if (autoIndex == 2)
            {
                table -> PutNumber("pipeline", Limelight::Cone);
                ty = table -> GetNumber("ty", 0.0);
                if (deadband(ty, 5) < 0)
                {
                    LLAlignTranslation = -deadband(ty-0.5, 5)/40;
                }
                else if (deadband(ty, 5))
                {
                    LLAlignTranslation = -deadband(ty+0.5, 5)/40;
                }
                else
                {
                    LLAlignTranslation = 0;
                }
                autoDesiredStates = swerveKinematics(LLAlignTranslation, 0, 0, navX.GetAngle(), false);
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
                
                intakePos = 30000;
                intakePercentage = (intakePos-IntakeUpDown.GetSelectedSensorPosition())/80000;
                if (intakePercentage < 0 && !intakeLimitSwitch.Get())
                {
                    intakePercentage = 0;
                }
                frc::SmartDashboard::PutNumber("Intake Error", intakePos-IntakeUpDown.GetSelectedSensorPosition());
                if (intakePos - IntakeUpDown.GetSelectedSensorPosition() < 1500 && !deadband(ty, 3))
                {
                    intakePercentage = 0;
                    autoIndex = 3;
                }
                IntakeUpDown.Set(TalonFXControlMode::PercentOutput, intakePercentage);
            }
            else if (autoIndex == 3){
                //cry
                
                if(heresyTimer > 40)
                {
                    FLSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    FRSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    BLSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    BRSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    FLDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    FRDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    BLDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    BRDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    
                    IntakeLeader.Set(0);

                    autoIndex = 4;
                }
                else
                {
                    heresyTimer++;
                    table -> PutNumber("pipeline", Limelight::Cone);
                    ty = table -> GetNumber("ty", 0.0);
                    if (deadband(ty, 5) < 0)
                    {
                        LLAlignTranslation = -deadband(ty-0.5, 5)/50;
                    }
                    else if (deadband(ty, 5))
                    {
                        LLAlignTranslation = -deadband(ty+0.5, 5)/50;
                    }
                    else
                    {
                        LLAlignTranslation = 0;
                    }

                    autoDesiredStates = swerveKinematics(LLAlignTranslation, 0.5, 0, navX.GetAngle(), false); // fmod(180-navX.GetAngle(), 360.0)/25
                    IntakeLeader.Set(0.6);
                    
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
            }
            else if (autoIndex == 4)
            {
                intakePos = 4500;
                intakePercentage = (intakePos-IntakeUpDown.GetSelectedSensorPosition())/50000;
                if (intakePos-IntakeUpDown.GetSelectedSensorPosition() < 0)
                {
                    intakePercentage = fmin(-0.1, intakePercentage);
                }
                else
                {
                    intakePercentage = fmax(0.1, intakePercentage);
                }
                if (intakePercentage < 0 && !intakeLimitSwitch.Get())
                {
                    intakePercentage = 0;
                }
                frc::SmartDashboard::PutNumber("Intake Error", intakePos-IntakeUpDown.GetSelectedSensorPosition());
                if (abs(intakePos - IntakeUpDown.GetSelectedSensorPosition()) < 500)
                {
                    autoIndex = 5;
                    intakePercentage = 0;
                    IntakeUpDown.Set(TalonFXControlMode::PercentOutput, 0);
                    heresyTimer = 0;
                }
                IntakeUpDown.Set(TalonFXControlMode::PercentOutput, intakePercentage);
            }
            else if (autoIndex == 5)
            {
                if (m_chooser.GetSelected() == kCubeBalanceLeft)
                {
                    moveToPosition(70, 190, 180);
                }
                else{
                    moveToPosition(-70, 190, 180);
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
            else
            {
                FLSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                FRSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                BLSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                BRSwerveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                FLDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                FRDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                BLDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                BRDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
                ShooterTop.Set(ControlMode::PercentOutput, 0);
                ShooterBottom.Set(ControlMode::PercentOutput, 0);
                IntakeLeader.Set(0);
                IntakeUpDown.Set(ControlMode::PercentOutput, 0);
                ArmMotor.Set(ControlMode::PercentOutput, 0);
            }
            if (!autoIndex)
            {
                armPos = -4000;
                armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
                if (!armLimitSwitch.Get())
                {
                    armPercentage = 0;
                }
                ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
                if(!ArmMotor.GetSelectedSensorVelocity() && !deadband(armPos-ArmMotor.GetSelectedSensorPosition(), 4))
                {
                    ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
                }
            }
            else if (autoIndex == 1)
            {
                armPos = -20000;
                armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
                if (!armLimitSwitch.Get())
                {
                    armPercentage = 0;
                }
                ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
                if(!ArmMotor.GetSelectedSensorVelocity() && !deadband(armPos-ArmMotor.GetSelectedSensorPosition(), 4))
                {
                    ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
                }
            }
            else if (autoIndex == 5)
            {
                IntakeUpDown.Set(TalonFXControlMode::PercentOutput, 0);
                ShooterTop.Set(ControlMode::PercentOutput, -0.4);
                ShooterBottom.Set(ControlMode::PercentOutput, -0.4);
                armPos = -4000;
                armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
                if (!armLimitSwitch.Get())
                {
                    armPercentage = 0;
                }
                ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
                if(!deadband(armPos-ArmMotor.GetSelectedSensorPosition(), 4))
                {
                    ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    heresyTimer++;
                    if (heresyTimer < 30)
                    {
                        IntakeLeader.Set(-0.2);
                    }
                    else
                    {
                        ShooterTop.Set(ControlMode::PercentOutput, 0);
                        ShooterBottom.Set(ControlMode::PercentOutput, 0);
                        IntakeLeader.Set(0);
                    }
                }
            }
            else
            {
                ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
            }

            if (autoIndex == 6)
            {
                balance = true;
                coneCollect = false;
            }
        }
        else if (cubeAutonomous) {
            autostop++;
            frc::SmartDashboard::PutNumber("Autostop", autostop);
            if (autostop < 4000)
            {
                armPos = -50000;
                armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
                if (!armLimitSwitch.Get())
                {
                    armPercentage = 0;
                }
                ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
                if(ArmMotor.GetSelectedSensorPosition()< -10000)
                {
                    autostop = 4000; // arbitrary impossible value
                }
            }
            else
            {
                if (autostop < 8000) // down
                {
                    armPos = -50000;
                    armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
                    if (!armLimitSwitch.Get())
                    {
                        armPercentage = 0;
                    }
                    ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
                    
                    intakePos = 10000;
                    intakePercentage = (intakePos-IntakeUpDown.GetSelectedSensorPosition())/50000;
                    frc::SmartDashboard::PutNumber("Intake Error", intakePos-IntakeUpDown.GetSelectedSensorPosition());
                    if (intakePos-IntakeUpDown.GetSelectedSensorPosition() < 0)
                    {
                        intakePercentage = fmin(-0.1, intakePercentage);
                    }
                    else
                    {
                        intakePercentage = fmax(0.1, intakePercentage);
                    }
                    if (abs(intakePos - IntakeUpDown.GetSelectedSensorPosition()) < 500 && autostop < 5000 && !deadband(armPos-ArmMotor.GetSelectedSensorPosition(), 4))
                    {
                        autostop = 5000;
                        ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
                    }
                    if (intakePercentage < 0 && !intakeLimitSwitch.Get())
                    {
                        intakePercentage = 0;
                    }
                    IntakeUpDown.Set(TalonFXControlMode::PercentOutput, intakePercentage);
                    if(autostop > 5020)
                    {
                        IntakeLeader.Set(-1);
                        if (autostop > 5040)
                        {
                            IntakeLeader.Set(0);
                            autostop = 8000;
                        }
                    }
                }
                else 
                {
                    intakePos = 1000;
                    intakePercentage = (intakePos-IntakeUpDown.GetSelectedSensorPosition())/40000;
                    frc::SmartDashboard::PutNumber("Intake Error", intakePos-IntakeUpDown.GetSelectedSensorPosition());
                    intakePercentage = fmin(-0.1, intakePercentage);
                    
                    if (abs(intakePos - IntakeUpDown.GetSelectedSensorPosition()) < 1500)
                    {
                        autostop = 12000;
                    }

                    if (intakePercentage < 0 && !intakeLimitSwitch.Get())
                    {
                        intakePercentage = 0;
                    }
                    IntakeUpDown.Set(TalonFXControlMode::PercentOutput, intakePercentage);
                    if (autostop > 11999)
                    {
                        IntakeUpDown.Set(TalonFXControlMode::PercentOutput, 0);
                        coneCollect = true;
                        cubeAutonomous = false;
                    }
                }
            }
        }
    } 
    
    else {
        // empty
    }

    

       
}

void Robot::TeleopInit() 
{
    mathConst::rotationVectorMultiplier = 0.4;
    limitSpeeds(FLDriveMotor, mathConst::speedLimit);
    limitSpeeds(FLSwerveMotor, mathConst::speedLimit);
    limitSpeeds(FRDriveMotor, mathConst::speedLimit);
    limitSpeeds(FRSwerveMotor, mathConst::speedLimit);
    limitSpeeds(BLDriveMotor, mathConst::speedLimit);
    limitSpeeds(BLSwerveMotor, mathConst::speedLimit);
    limitSpeeds(BRDriveMotor, mathConst::speedLimit);
    limitSpeeds(BRSwerveMotor, mathConst::speedLimit);
    mathConst::pseudoSpeedLimit = mathConst::speedLimit;
    navX.ZeroYaw();
    navX.SetAngleAdjustment(180.0);
    IntakeUpDown.SetNeutralMode(NeutralMode::Brake);
    ArmMotor.ConfigPeakOutputReverse(-0.3);
}
void Robot::TeleopPeriodic()
{
    frc::SmartDashboard::PutNumber("rotvect", mathConst::rotationVectorMultiplier);

    ArmOutCurr = fmax(ArmMotor.GetOutputCurrent(), ArmOutCurr);
    frc::SmartDashboard::PutNumber("Arm Output Current", ArmOutCurr);
    
    IntakeOutCurr = fmax(IntakeUpDown.GetOutputCurrent(), IntakeOutCurr);
    frc::SmartDashboard::PutNumber("Intake Output Current", IntakeOutCurr);

    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    ty = table -> GetNumber("ty", 0.0);
    frc::SmartDashboard::PutNumber("ty", ty);
    
    if (controller.GetRightBumper()) {
        if (deadband(ty, 5) < 0)
        {
            LLAlignTranslation = -deadband(ty-0.5, 5)/30;
        }
        else if (deadband(ty, 5))
        {
            LLAlignTranslation = -deadband(ty+0.5, 5)/30;
        }
        else
        {
            LLAlignTranslation = 0;
        }
        // alignSouth = deadband(fmod((180-navX.GetAngle()), 360.0), 3)/8;
        // if (alignSouth)
        // {
        //     if (deadband(alignSouth*16, 1))
        //     {
        //         alignSouth = alignSouth*2;
        //     }
        //     moduleDesiredStates = swerveKinematics(0, 0, alignSouth, navX.GetAngle(), false);
        // }
        // else 
        // {
            moduleDesiredStates = swerveKinematics(LLAlignTranslation, 0, 0, navX.GetAngle(), false);
        // }
    }
    else if (controller.GetAButton())
    {
        // ALIGN EAST
        if (allianceColor == Alliances::Red){
            alignEast = deadband(fmod((90-navX.GetAngle()), 360.0), 3);    
        }
        else if (allianceColor == Alliances::Blue)
        {
            alignEast = deadband(fmod((-90-navX.GetAngle()), 360.0), 3);
        }
        
        if (!deadband(controller.GetLeftX(), 0) && !deadband(controller.GetLeftY(), 0))
        {
            alignEast = alignEast / 5;
        }
        else
        {
            alignEast = alignEast*0.05*magnitude(deadband(controller.GetLeftX(), 0), deadband(controller.GetLeftY(), 0));
        }
        frc::SmartDashboard::PutNumber("AlignEast", alignEast);
        moduleDesiredStates = swerveKinematics(deadband(controller.GetLeftX(), 0), deadband(controller.GetLeftY(), 0), alignEast, navX.GetAngle(), false);
    }
    else {
        moduleDesiredStates = swerveKinematics(deadband(controller.GetLeftX(), 0), deadband(controller.GetLeftY(), 0), deadband(controller.GetRightX(), 0), navX.GetAngle(), false);
    }

    // //Intake's intake mechanism
    if(controller.GetRightTriggerAxis())
    {
        IntakeLeader.Set(0.5);
    }
    else if (controller.GetBButton())
    {
        IntakeLeader.Set(-1);
        // if (IntakeUpDown.GetSelectedSensorPosition() > 25000)
        // {
        //     IntakeLeader.Set(-1);    
        // }
        // else
        // {
        //     IntakeLeader.Set(-1);
        // }
    }
    else if (controllerAux.GetBButton())
    {
        IntakeLeader.Set(-0.5);
    }
    else
    {
      IntakeLeader.Set(0);
    }
    //Intake up down mechanism
    frc::SmartDashboard::PutNumber("Intake desired position", intakePos);
    frc::SmartDashboard::PutNumber("Intake sensor position", IntakeUpDown.GetSelectedSensorPosition());

    frc::SmartDashboard::PutNumber("Arm desired position", armPos);
    frc::SmartDashboard::PutNumber("Arm sensor position", ArmMotor.GetSelectedSensorPosition());

    if (ArmMotor.GetOutputCurrent()>35 || !armLimitSwitch.Get())
    {
        ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
        armState = 0;
        armPos = ArmMotor.GetSelectedSensorPosition();
    }
    else if (armState)
    {
        switch (armState)
        {
            case 1:
                armPos = -77000;
                break;

            case 2:
                armPos = -53000;
                break;

            case 3:
                armPos = -30000; // calibrate
                break;

            case 4:
                armPos = -3000;
                break;

            case 5:
                armPos = -48000;
                break;

            default:
                break;
        }
        armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
                
        if (armPercentage > 0)
        {
            // speed limiting for retracting arm (prevents colliding into superstructure)
            double armSpeedIntermediate = fmin(0.3, -0.3*ArmMotor.GetSelectedSensorPosition()/8000);

            armPercentage = fmin(armPercentage, armSpeedIntermediate);
        }

        ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
        if(!deadband(ArmMotor.GetSelectedSensorVelocity(), 2))
        {
            armState = 0;
        }
    }
    else
    {
        if (controllerAux.GetLeftBumper() && armLimitSwitch.Get()){
            ArmMotor.Set(ControlMode::PercentOutput, fmin(0.2, -0.2*ArmMotor.GetSelectedSensorPosition()/8000));
        }
        else if (controllerAux.GetRightBumper()){
            ArmMotor.Set(ControlMode::PercentOutput, -0.2);
            // Dojo test
            // fmax(-0.3, -0.3*69000/abs(ArmMotor.GetSelectedSensorPosition()))
        }
        else {
            ArmMotor.Set(ControlMode::PercentOutput, 0);
        }
        if (controllerAux.GetPOV()==0)
        {
            armState = 1;
            heightToGoal = Limelight::tallHeightInches - Limelight::limelightLensHeightInches;
            table -> PutNumber("pipeline", Limelight::TopReflective);
        }
        else if (controllerAux.GetPOV()==90)
        {
            armState = 2;
            heightToGoal = Limelight::midHeightInches - Limelight::limelightLensHeightInches;
            table -> PutNumber("pipeline", Limelight::BottomReflective);
        }
        else if (controllerAux.GetPOV()==270)
        {
            armState = 5;
        }
        
        frc::SmartDashboard::PutBoolean("IntakeActive", intakeActive);

        frc::SmartDashboard::PutNumber("intake stage", intakeStage);

        if (controllerAux.GetAButton())
        {
            if (!intakeStage && ArmMotor.GetSelectedSensorPosition() > (-18000))
            {
                armState = 3;
            }
            else
            {
                if (IntakeUpDown.GetSelectedSensorPosition()<4500 && IntakeUpDown.GetSelectedSensorPosition()>3500)
                {
                    if (ArmMotor.GetSelectedSensorPosition()>(-6000) && !armState)
                    {
                        intakeStage++;
                        if (intakeStage < 50)
                        {
                            ShooterTop.Set(ControlMode::PercentOutput, -0.4);
                            ShooterBottom.Set(ControlMode::PercentOutput, -0.4);
                            IntakeLeader.Set(-0.2);
                        }
                        else
                        {
                            intakeStage = 0;
                        }
                    }
                    else
                    {
                        intakeStage = 2;
                        armState = 4;
                        ShooterTop.Set(ControlMode::PercentOutput, -0.2);
                        ShooterBottom.Set(ControlMode::PercentOutput, -0.2);
                    }
                }
                else
                {
                    intakePos = 5500;
                    intakeStage = 1;
                }
            }
        }
        else
        {
            intakeStage = 0;
        }
    }
    // if (intakeActive)
    // {
    //     if (IntakeUpDown.GetOutputCurrent()>37)
    //     {
    //         intakePercentage = 0;
    //         intakePos = IntakeUpDown.GetSelectedSensorPosition();
    //     }
    //     else // down
    //     {
    //         intakePercentage = (intakePos-IntakeUpDown.GetSelectedSensorPosition())/50000;
    //         frc::SmartDashboard::PutNumber("Intake Error", intakePos-IntakeUpDown.GetSelectedSensorPosition());
    //     }
    //     if (intakePos-IntakeUpDown.GetSelectedSensorPosition() < 0)
    //     {
    //         intakePercentage = fmin(-0.1, intakePercentage);
    //     }
    //     frc::SmartDashboard::PutNumber("percentage intake", intakePercentage);
    //     IntakeUpDown.Set(TalonFXControlMode::PercentOutput, intakePercentage);
    //     if(!deadband(intakePos-IntakeUpDown.GetSelectedSensorPosition(), 4))
    //     {
    //         intakeActive = false;
    //         IntakeUpDown.Set(TalonFXControlMode::PercentOutput, 0);
    //     }
    // }
    // else
    // {
    //     if(controller.GetYButtonPressed()){ // up
    //         intakePos = 1700;
    //         intakeActive = true;
    //     }
    //     else if (controller.GetXButtonPressed()){ // down
    //         intakePos = 19500;
    //         intakeActive = true;
    //     }
    //     else if (controllerAux.GetLeftTriggerAxis()>0.5)
    //     {
    //         intakePos = 6000;
    //         intakeActive = true;
    //     }
    // }

    if (IntakeUpDown.GetOutputCurrent()>37)
    { 
        intakePercentage = 0;
        intakePos = IntakeUpDown.GetSelectedSensorPosition();
    }
    else // down
    {
        intakePercentage = (intakePos-IntakeUpDown.GetSelectedSensorPosition());
        frc::SmartDashboard::PutNumber("Intake Error", intakePos-IntakeUpDown.GetSelectedSensorPosition());
        if (abs(intakePercentage) < 200)
        {
            if (intakeLimitSwitch.Get())
            {
                intakePercentage = -0.03;
            }
            else
            {
                intakePercentage = 0;
            }
        }
        else if (intakePercentage < 0 && intakeLimitSwitch.Get())
        {
            intakePercentage = fmin(-0.1, intakePercentage / 50000);
        }
        else if (intakePercentage > 0)
        {
            intakePercentage = fmax(0.1, intakePercentage / 50000);
        }
        else
        {
            intakePercentage = 0;
        }
        
    }
    frc::SmartDashboard::PutNumber("percentage intake", intakePercentage);
    IntakeUpDown.Set(TalonFXControlMode::PercentOutput, intakePercentage);

    if(controller.GetYButtonPressed()){ // up
        intakePos = 0;
        intakeActive = true;
    }
    else if (controller.GetXButtonPressed()){ // down
        intakePos = 30000;
        intakeActive = true;
    }
    else if (controllerAux.GetLeftTriggerAxis()>0.5)
    {
        intakePos = 10000;
        intakeActive = true;
    }
    else if (controller.GetLeftBumper())
    {
        intakePos = 4000;
    }

    if (controllerAux.GetXButton()){
        ShooterTop.Set(ControlMode::PercentOutput, -0.5);
        ShooterBottom.Set(ControlMode::PercentOutput, -0.5);
        // IntakeLeader.Set(-0.2);
    }
    else if (controllerAux.GetYButton()){
        ShooterTop.Set(ControlMode::PercentOutput, 0.9);
        ShooterBottom.Set(ControlMode::PercentOutput, 1);
    }
    else {
        ShooterTop.Set(ControlMode::PercentOutput, 0);
        ShooterBottom.Set(ControlMode::PercentOutput, 0);
    }    
    
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
    // logSwerveNumber("drive", moduleDesiredStates.flm,moduleDesiredStates.frm,moduleDesiredStates.blm,moduleDesiredStates.brm);
    
    // Desired turn angles (degrees)
    logSwerveNumber("Desired Angle", desiredTurnFL, desiredTurnFR, desiredTurnBL, desiredTurnBR);

    // CANCoder Absolute Readings
    // logSwerveNumber("CANCoder", FLCANCoder.GetAbsolutePosition(), FRCANCoder.GetAbsolutePosition(), BLCANCoder.GetAbsolutePosition(), BRCANCoder.GetAbsolutePosition());

    // Gyro angle (degrees)
    frc::SmartDashboard::PutNumber("Angle", navX.GetAngle());
    frc::SmartDashboard::PutNumber("Roll", navX.GetRoll());
    
    frc::SmartDashboard::PutNumber("Gyro Zero", navX.GetAngleAdjustment());
    
    // Zero gyro (press d-pad in whatever direction the PDP is relative to the North you want)
    if (controller.GetPOV()!=-1)
    {
        navX.ZeroYaw();
        navX.SetAngleAdjustment(controller.GetPOV());
        
    }
    
    if (controller.GetPOV()!=-1)
    {
        LED.Set(0.3); // yellow
    }
    else if (abs(ty)<=1 && ty != 0 && !FLDriveMotor.GetSelectedSensorVelocity())
    {
        LED.Set(-0.69); // rainbow
    }
    else
    {
        LED.Set(0.99); // black
    }

    if(controller.GetLeftTriggerAxis())
    {
        if(throttle==false)
        {
            limitSpeeds(FLDriveMotor, 0.5);
            limitSpeeds(FLSwerveMotor, 0.5);
            limitSpeeds(FRDriveMotor, 0.5);
            limitSpeeds(FRSwerveMotor, 0.5);
            limitSpeeds(BLDriveMotor, 0.5);
            limitSpeeds(BLSwerveMotor, 0.5);
            limitSpeeds(BRDriveMotor, 0.5);
            limitSpeeds(BRSwerveMotor, 0.5);
            throttle = true;
        }
    }
    else
    {
        if(throttle==true)
        {
            limitSpeeds(FLDriveMotor, mathConst::speedLimit);
            limitSpeeds(FLSwerveMotor, mathConst::speedLimit);
            limitSpeeds(FRDriveMotor, mathConst::speedLimit);
            limitSpeeds(FRSwerveMotor, mathConst::speedLimit);
            limitSpeeds(BLDriveMotor, mathConst::speedLimit);
            limitSpeeds(BLSwerveMotor, mathConst::speedLimit);
            limitSpeeds(BRDriveMotor, mathConst::speedLimit);
            limitSpeeds(BRSwerveMotor, mathConst::speedLimit);
            throttle = false;
        }
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