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

// Colour Sensor
rev::ColorSensorV3 m_colorSensor{frc::I2C::Port::kOnboard};

int autostop;

// true = going down; false = going up
bool intakeState = true;
double intakePos; 
double intakePercentage;

// true = set position; 0 = manual
int armState = 0;
double armPos;
double armPercentage;


// wall of not constant variable shame 
// Limelight shenanigans
auto table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
double tx;
double ty;
std::vector<double> aprilPos;

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

// auto
double diffX;
double diffY;
double diffAngle;
double linearDisplacement;
double translationAngle;
int autoIndex = 0;

bool balance = true;
bool onChargingStation = false;

// colour
rev::ColorMatch m_colorMatcher;
frc::Color detectedColor;
std::string colorstring;
frc::Color matchedColor;
double confidence;

bool throttle;
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

frc::Pose2d currentPose{0_in, 0_in, 0_deg};

// Input degrees
// Automatically applies deadbands
// Outputs a swerveModule object 
// (percentage speeds and angles in the order FL, FR, BL, BR)
desiredSwerveModule swerveKinematics(double xLeft, double yLeft, double xRight, double gyro, bool applyExponentTrue)
{
    // cmath reads radians; applies deadbands
    gyro = getRadian(gyro);
    
    Point posVector = Point(0.0, 0.0);
    
    double joystickMagnitude=1;
    if (ArmMotor.GetSelectedSensorPosition())
    {
        joystickMagnitude = fmin(1, -20000/ArmMotor.GetSelectedSensorPosition());
    }
    
    double exponentialModification = magnitude(xLeft, yLeft);
    
    if (applyExponentTrue)
    {
        exponentialModification = exponentiate(exponentialModification, mathConst::ThrottleMode);
    } 

    joystickMagnitude = joystickMagnitude*exponentialModification;
    
    // Creates a unit vector multiplied by right joystick input and proportionally scaled by "rotationVectorMultiplier"
    double rotationScalar=0;
    if(xRight)
    {
        rotationScalar = pow(xRight, 2)*xRight/abs(xRight);
    }
    if (joystickMagnitude)  // Check if left joystick has an input
    {
        // atan2 converts joystick input into angle
        // + gyro makes desired angle calculation field relative 
        double fieldRelativePosAngle = atan2(xLeft, yLeft) + gyro;
        
        // convert angles back into Cartesian
        posVector.x = joystickMagnitude * sin(fieldRelativePosAngle);
        posVector.y = joystickMagnitude * cos(fieldRelativePosAngle);
        rotationScalar = mathConst::rotationVectorMultiplier * rotationScalar; 
    }
    else if (!xRight) // the < 0.1 is another reduncancy that is within the deadband
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
    
    // pure turning
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
    diffAngle = fmod(autonomous::kAutoCommandList[autoIndex][2] - navX.GetYaw(), 360.0);
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

    linearDisplacement = deadband(linearDisplacement, 1) / 40;  // 40 inches is the Kp for now 
                                                                //* using limelight distance allowance for this too
    if (!linearDisplacement)
    {
        autoDesiredStates = swerveKinematics(0, 0, diffAngle/25, 0, false); // 25 is an arbitrary Kp again
        if (!deadband(diffAngle, 2))
        {
            autoIndex++;
        }
    }
    else
    {
        autoDesiredStates = swerveKinematics(linearDisplacement*sin(translationAngle), linearDisplacement*cos(translationAngle), linearDisplacement, navX.GetYaw(), false);
    }
    // frc::SmartDashboard::PutNumber("auto x", linearDisplacement*sin(translationAngle));
    // frc::SmartDashboard::PutNumber("auto y", linearDisplacement*cos(translationAngle));
    // frc::SmartDashboard::PutNumber("auto xRight", linearDisplacement);
    // frc::SmartDashboard::PutNumber("auto rotation", mathConst::rotationVectorMultiplier);
}

void Robot::RobotInit() 
{
    processBaseDimensions(mathConst::xCoords, mathConst::yCoords);

    m_colorMatcher.AddColorMatch(Colours::KYellowTarget);
    m_colorMatcher.AddColorMatch(Colours::KPurpleTarget);

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
    IntakeUpDown.SetNeutralMode(NeutralMode::Coast);

    navX.ZeroYaw();
    
    ArmMotor.ConfigPeakOutputReverse(-0.3);
    ShooterTop.ConfigPeakOutputReverse(-1);
    ShooterBottom.ConfigPeakOutputReverse(-1);
    IntakeUpDown.ConfigPeakOutputReverse(-0.2);
    ArmMotor.ConfigPeakOutputForward(0.3);
    ShooterTop.ConfigPeakOutputForward(1);
    ShooterBottom.ConfigPeakOutputForward(1);
    IntakeUpDown.ConfigPeakOutputForward(0.2);

    IntakeUpDown.SetSelectedSensorPosition(0);
    ArmMotor.SetSelectedSensorPosition(0);
}
void Robot::RobotPeriodic() 
{
    detectedColor = m_colorSensor.GetColor();
    matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);
    if(matchedColor == Colours::KYellowTarget) {
        colorstring = "Yellow";
    }
    else if (matchedColor == Colours::KPurpleTarget){
        colorstring = "Purple";
    }
    // frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    // frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    // frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    // frc::SmartDashboard::PutNumber("Confidence", confidence);
    // frc::SmartDashboard::PutString("Detected Color", colorstring);
    uint32_t proximity = m_colorSensor.GetProximity();
    // frc::SmartDashboard::PutNumber("Proximity", proximity);
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

    limitSpeeds(FLDriveMotor, 0.5);
    limitSpeeds(FLSwerveMotor, 0.5);
    limitSpeeds(FRDriveMotor, 0.5);
    limitSpeeds(FRSwerveMotor, 0.5);
    limitSpeeds(BLDriveMotor, 0.5);
    limitSpeeds(BLSwerveMotor, 0.5);
    limitSpeeds(BRDriveMotor, 0.5);
    limitSpeeds(BRSwerveMotor, 0.5);
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
    frc::SmartDashboard::PutNumber("X ", currentPose.X().value()*39.37);
    frc::SmartDashboard::PutNumber("Y ", currentPose.Y().value()*39.37);

    if(balance){
        double roll = deadband(navX.GetRoll(), 3);
        if (abs(roll)>8)
        {
            onChargingStation = true;
            limitSpeeds(FLDriveMotor, 0.2);
            limitSpeeds(FLSwerveMotor, 0.2);
            limitSpeeds(FRDriveMotor, 0.2);
            limitSpeeds(FRSwerveMotor, 0.2);
            limitSpeeds(BLDriveMotor, 0.2);
            limitSpeeds(BLSwerveMotor, 0.2);
            limitSpeeds(BRDriveMotor, 0.2);
            limitSpeeds(BRSwerveMotor, 0.2);
        }
        if (onChargingStation)
        {
            roll = -roll / 150;
        }
        else{
            roll = 1;
        }
        autoDesiredStates = swerveKinematics(0, roll, 0, navX.GetYaw(), false);
    }
    else {
        autostop++;
        if (autostop < 20)
        {
            ShooterTop.Set(ControlMode::PercentOutput, -0.2);
            ShooterBottom.Set(ControlMode::PercentOutput, -0.2);
            IntakeLeader.Set(-0.2);
        }
        else
        {
            if (autostop < 4020)
            {
                armPos = -53000;
                ShooterTop.Set(ControlMode::PercentOutput, -0.05);
                ShooterBottom.Set(ControlMode::PercentOutput, -0.05);
                armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
                ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
                if(ArmMotor.GetSelectedSensorVelocity()==0)
                {
                    autostop = 4000; // arbitrary impossible value
                }
            }
            else if (autostop < 4040)
            {
                ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
                ShooterTop.Set(ControlMode::PercentOutput, 0.9);
                ShooterBottom.Set(ControlMode::PercentOutput, 1);
            }
            else{
                balance = true;
                ShooterTop.Set(ControlMode::PercentOutput, 0);
                ShooterBottom.Set(ControlMode::PercentOutput, 0);
            }

        }
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
    navX.ZeroYaw();
    navX.SetAngleAdjustment(180.0);
}
void Robot::TeleopPeriodic()
{
    frc::SmartDashboard::PutNumber("rotvect", mathConst::rotationVectorMultiplier);
    // moduleDesiredStates contains all calculated desired values.
    // .flm is "Front left magnitude" (percentage)
    // .fla is "Front left angle" (degrees)
    // fl[], fr[], bl[], br[]
    
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    tx = table -> GetNumber("tx", 0.0);
    ty = table -> GetNumber("ty", 0.0);
    // frc::SmartDashboard::PutNumber("tx", tx);
    frc::SmartDashboard::PutNumber("ty", ty);
    
    // angleToGoalDegrees = Limelight::limelightMountAngleDegrees + tx;
    // distanceFromLimelightToGoalInches = heightToGoal/tan(getRadian(angleToGoalDegrees));

    // frc::SmartDashboard::PutNumber("Limelight Distance", LimelightDifference);

    if (controller.GetRightBumper()) {
        //Retroreflective Tape Tracking
        // table -> PutNumber("pipeline", 1);
        // LimelightDifference = deadband(distanceFromLimelightToGoalInches-45, 1); // 45 = desired
    
        // if (abs(LimelightDifference)>20){
        //     LimelightDifference = LimelightDifference/abs(LimelightDifference);
        // }
        // else {
        //     LimelightDifference = LimelightDifference/20;
        // }
        // LimelightSlew = slew(LimelightSlew, LimelightDifference, 1);
        // frc::SmartDashboard::PutNumber("LimelightDifference", LimelightSlew);

        moduleDesiredStates = swerveKinematics(deadband(ty, 0)/50, 0, 0, navX.GetYaw(), false);
       

        // frc::SmartDashboard::PutNumber("Limelight", LimelightSlew);
        
        
    }
    else if (controller.GetLeftBumper())
    {
        //Apriltag tracking
        table -> PutNumber("pipeline", 2);
         
        // 180 is perpendicular to tags
        // int desiredRotation = 180;
        // LimelightDifference = deadband(distanceFromLimelightToGoalInches-45, 1); // 45 = desired
    
        // if (abs(LimelightDifference)>20){
        //     LimelightDifference = LimelightDifference/abs(LimelightDifference);
        // }
        // else {
        //     LimelightDifference = LimelightDifference/20;
        // }
        // LimelightSlew = slew(LimelightSlew, LimelightDifference, 1);
        // frc::SmartDashboard::PutNumber("LimelightDifference", LimelightSlew);
        // moduleDesiredStates = swerveKinematics(-deadband(desiredRotation-fmod(navX.GetYaw()+1080.0, 360.0), 2)/20, 0, 0, navX.GetYaw(), false);
    }
    else if (controllerAux.GetAButton())
    {
        frc::SmartDashboard::PutNumber("align to 180", fmod((0-navX.GetYaw()), 360.0)/30);
        moduleDesiredStates = swerveKinematics(0, 0, fmod((0-navX.GetYaw()), 360.0)/30, 0, false);
    }
    else {
        moduleDesiredStates = swerveKinematics(deadband(controller.GetLeftX(), 0), deadband(controller.GetLeftY(), 0), deadband(controller.GetRightX(), 0), navX.GetAngle(), true);
    }
    // intake mechanism
    if(controller.GetRightTriggerAxis())
    {
      IntakeLeader.Set(0.5);
    }
    else if (controller.GetBButton())
    {
      IntakeLeader.Set(-0.4);
    }
    else
    {
      IntakeLeader.Set(0);
    }
    //Intake up down mechanism
    frc::SmartDashboard::PutNumber("intake position", intakePos);
    frc::SmartDashboard::PutNumber("sensor pos", IntakeUpDown.GetSelectedSensorPosition());

    frc::SmartDashboard::PutNumber("arm position", armPos);
    frc::SmartDashboard::PutNumber("arm sensor pos", ArmMotor.GetSelectedSensorPosition());
    
    if (ArmMotor.GetOutputCurrent()>35)
    {
        ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
        armState = 0;
        armPos = ArmMotor.GetSelectedSensorPosition();
    }
    else if (armState)
    {
        if (armState == 1)
        {
            armPos = -77000;
        }
        else
        {
            armPos = -53000;
        }
        armPercentage = (armPos-ArmMotor.GetSelectedSensorPosition())/10000;
        ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
        if(!ArmMotor.GetSelectedSensorVelocity())
        {
            armState = 0;
        }
    }
    else
    {
        if (controllerAux.GetLeftBumper()){
            ArmMotor.Set(ControlMode::PercentOutput, fmin(0.3, -0.3*ArmMotor.GetSelectedSensorPosition()/8000));
        }
        else if (controllerAux.GetRightBumper()){
            ArmMotor.Set(ControlMode::PercentOutput, -0.3);
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
            table -> PutNumber("pipeline", 1);
        }
        else if (controllerAux.GetPOV()==90 || controllerAux.GetPOV()==270)
        {
            armState = 2;
            heightToGoal = Limelight::midHeightInches - Limelight::limelightLensHeightInches;
            table -> PutNumber("pipeline", 3);
        }
    }
    
    if(controller.GetYButtonPressed()){
        intakeState = false;
        intakePos = 500;
    }
    else if (controller.GetXButtonPressed()){
        intakePos = 29000;
        intakeState = true;
    }
        
    if (IntakeUpDown.GetOutputCurrent()>30)
    {
        intakePercentage = 0;
        intakePos = IntakeUpDown.GetSelectedSensorPosition();
    }
    else if(intakeState)
    {
        intakePercentage = (intakePos-IntakeUpDown.GetSelectedSensorPosition())/15000;
    }
    else{
        intakePercentage = (intakePos-IntakeUpDown.GetSelectedSensorPosition())/60000;
    }

    frc::SmartDashboard::PutNumber("percentage intake", intakePercentage);
    IntakeUpDown.Set(TalonFXControlMode::PercentOutput, intakePercentage);

    if (controllerAux.GetXButton()){
        ShooterTop.Set(ControlMode::PercentOutput, -0.2);
        ShooterBottom.Set(ControlMode::PercentOutput, -0.2);
        IntakeLeader.Set(-0.2);
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
    logSwerveNumber("drive", moduleDesiredStates.flm,moduleDesiredStates.frm,moduleDesiredStates.blm,moduleDesiredStates.brm);
    
    // Desired turn angles (degrees)
    logSwerveNumber("Desired Angle", desiredTurnFL, desiredTurnFR, desiredTurnBL, desiredTurnBR);

    // CANCoder Absolute Readings
    logSwerveNumber("CANCoder", FLCANCoder.GetAbsolutePosition(), FRCANCoder.GetAbsolutePosition(), BLCANCoder.GetAbsolutePosition(), BRCANCoder.GetAbsolutePosition());

    // Gyro angle (degrees)
    frc::SmartDashboard::PutNumber("Yaw", navX.GetAngle());
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
    else if (abs(ty)<=2 && ty != 0 && !FLDriveMotor.GetSelectedSensorVelocity())
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