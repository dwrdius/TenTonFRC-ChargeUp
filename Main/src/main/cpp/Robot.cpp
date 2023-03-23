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

// Limit switches
frc::DigitalInput armLimitSwitch{0};
frc::DigitalInput intakeLimitSwitch{1};

// Peak current readings (safety)
double ArmOutCurr = 0; 
double IntakeOutCurr = 0;
double NeoOutCurr = 0;

// Arbitrary timers
int autostop;
int heresyTimer = 0;

// Intake Variables
double intakePos; // 0 - 27000
double intakePercentage; // Intake motor drive percentage
int intakeStage; 
bool IntakeAtPosition = false; // Intake within acceptable range (1500 TalonFX Units)

// Arm Variables
int armState = 0; // 0: TeleOp, 1: Top, 2: Middle, 3: Intermediate, 4: Cone Handoff, 5: Substation
double armPos;
double armPercentage;
bool ArmAtPosition = false; // Arm within acceptable range (1000 TalonFX Units)

// Limelight  
auto table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); // Declare Limelight

double ty; // Horizontal offset for alignment (LL) (degrees)

double LLAlignTranslation; // Intermediate variable for LL alignment

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
double diffX; // x offset cartesian (inches)
double diffY; // y offset cartesian (inches)
double diffAngle; // intermediate for angle error (degrees)
double linearDisplacement; // intermediate for translation error (degrees)
double translationAngle; // angle for movement (straight line translation)
int autoIndex = 0; // "which step of the auto are you at" (used wrong sometimes)

// autonomous triggers (in order of if / else)
bool balance = false; // auto balance using navX roll
bool collectCone = false; // collecting cone and returning
bool cubeAutonomous = true; // shooting a cube (top) 

// auto intermediate triggers
bool onChargingStation = false; // on charge station (throttle speed for balance)
bool DrivetrainActive = false; // enable drive

bool throttle = false; // Throttle speed to 50%

double alignEast; // PF loop for aligning east
double alignSouth; // PF loop for aligning south

// Intake encoders (for debugging velocity)
auto neo13 = IntakeFollower.GetEncoder();
auto neo14 = IntakeLeader.GetEncoder();

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

    if(xRight) // prevents /0
    {
        rotationScalar = pow(xRight, 2)*xRight/abs(xRight); // x^2 right joystick values
    }

    double joystickMagnitude = 1; // limiter for left joystick (arm position)
    
    Point posVector = Point(0.0, 0.0);

    // if arm position is larger than 20000, slow down driving
    if (ArmMotor.GetSelectedSensorPosition())
    {
        joystickMagnitude = fmin(1, abs(20000/ArmMotor.GetSelectedSensorPosition()));
    }

    double exponentialModification = magnitude(xLeft, yLeft); // multi-use variable again
    
    // if exponential boolean is true, apply selected exponential transformation (see helper.h and desmos)
    if (exponentialTrue)
    {
        exponentialModification = exponentiate(exponentialModification, 2); 
    } 
    
    joystickMagnitude = joystickMagnitude*exponentialModification; // combine exponential and arm limiters
    
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
    
    // if no translation, limit the speed
    if(!exponentialModification)
    {
        limitingScalar = limitingScalar * 5; // 5 is arbitrary slow down
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
// move to coordinate at index (x, y, angle)
// takes no arguments; modify index in autonomous.h
void moveToCoord()
{ 
    diffX = -(autonomous::kAutoCommandList[autoIndex][0] + currentPose.Y().value()*39.37);
    diffY = (autonomous::kAutoCommandList[autoIndex][1] - currentPose.X().value()*39.37);
    diffAngle = fmod(autonomous::kAutoCommandList[autoIndex][2] - navX.GetAngle(), 360.0);
    translationAngle = atan2(diffX, diffY);

    constrainToPlusMinus180(diffAngle);

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

// ___________________________________________________

void TranslateAlign (int pipeline)
{
    table -> PutNumber("pipeline", pipeline);
    ty = table -> GetNumber("ty", 0.0);
    if (ty < 0)
    {
        LLAlignTranslation = -deadband(ty-0.5, 5)/20;
    }
    else
    {
        LLAlignTranslation = -deadband(ty+0.5, 5)/20;
    }
    autoDesiredStates = swerveKinematics(LLAlignTranslation, 0, 0, navX.GetAngle(), false);
}

void IntakePosition (int position)
{
    intakePos = position;
    double intakeError = intakePos-IntakeUpDown.GetSelectedSensorPosition();
    
    intakePercentage = (intakeError)/80000;

    if (intakePercentage < 0 && !intakeLimitSwitch.Get())
    {
        intakePercentage = 0;
    }
    else if (IntakeUpDown.GetOutputCurrent() > 35)
    {
        intakePercentage = 0;
    }
    if (intakeError < 1500)
    {
        intakePercentage = 0;
        IntakeAtPosition = true;
    }
    else
    {
        IntakeAtPosition = false;
    }
    
    frc::SmartDashboard::PutNumber("Intake Error", intakeError);
    frc::SmartDashboard::PutNumber("Intake Desired Position", intakePos);
    frc::SmartDashboard::PutNumber("Intake Current Position", IntakeUpDown.GetSelectedSensorPosition());
    frc::SmartDashboard::PutBoolean("Intake At Position", IntakeAtPosition);
}

void maintainIntakePosition ()
{
    double intakeError = intakePos-IntakeUpDown.GetSelectedSensorPosition();
    if (abs(intakeError) < 500)
    {
        intakePercentage = -0.065;
    }
    else if (intakeError < 0)
    {
        intakePercentage = fmin(-0.1, intakeError / 50000);
    }
    else
    {
        intakePercentage = 0;
    }
}

void SetIntakePosition (int position, bool exitCondition)
{
    if (IntakeAtPosition)
    {
        maintainIntakePosition();
    }
    else
    {
        IntakePosition (position);
    }

    if (!exitCondition)
    {
        IntakeUpDown.Set(TalonFXControlMode::PercentOutput, intakePercentage);
    }
}

void ArmPosition (int position)
{
    double armError = armPos-ArmMotor.GetSelectedSensorPosition();
    armPos = position;
    armPercentage = armError/10000;
    
    if (!armLimitSwitch.Get())
    {
        armPercentage = 0;
    }
    if(!deadband(armError, 4))
    {
        armPercentage = 0;
        ArmAtPosition = true;
    }

    frc::SmartDashboard::PutNumber("Arm Error", armError);
    frc::SmartDashboard::PutNumber("Arm Desired Position", armPos);
    frc::SmartDashboard::PutNumber("Arm Current Position", ArmMotor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutBoolean("Arm At Position", ArmAtPosition);
}

void SetArmPosition (int position)
{
    if (!ArmAtPosition)
    {
        ArmPosition (position); // write actual position

        ArmMotor.Set(TalonFXControlMode::PercentOutput, armPercentage);
    }
    else
    {
        ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
    }
}

// ___________________________________________________

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
    limitSpeeds(ShooterTop, 1);
    limitSpeeds(ShooterBottom, 1);
    
    IntakeUpDown.ConfigPeakOutputReverse(-0.5);
    IntakeUpDown.ConfigPeakOutputForward(0.15);

    IntakeUpDown.SetSelectedSensorPosition(0);
    ArmMotor.SetSelectedSensorPosition(0);
}
void Robot::RobotPeriodic() 
{
    frc::SmartDashboard::PutNumber("Arm Limit", armLimitSwitch.Get());
            
    frc::SmartDashboard::PutNumber("Neo14", neo14.GetVelocity());
    frc::SmartDashboard::PutNumber("Neo13", neo13.GetVelocity());

    frc::SmartDashboard::PutBoolean("Arm Limit", armLimitSwitch.Get());
    frc::SmartDashboard::PutBoolean("Intake Limit", intakeLimitSwitch.Get());
    if (!armLimitSwitch.Get())
    {
        ArmMotor.SetSelectedSensorPosition(500);
    }
    if (!intakeLimitSwitch.Get())
    {
        IntakeUpDown.SetSelectedSensorPosition(0);
    }

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

    ArmOutCurr = fmax(ArmMotor.GetOutputCurrent(), ArmOutCurr);
    frc::SmartDashboard::PutNumber("Arm Output Current", ArmOutCurr);
    
    IntakeOutCurr = fmax(IntakeUpDown.GetOutputCurrent(), IntakeOutCurr);
    frc::SmartDashboard::PutNumber("Intake Output Current", IntakeOutCurr);

    NeoOutCurr = fmax(IntakeLeader.GetOutputCurrent(), NeoOutCurr);
    frc::SmartDashboard::PutNumber("NEO Output Current", NeoOutCurr);
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

    LimitDrivetrainSpeeds(0.2);

    autostop = 0;
}

void Robot::AutonomousPeriodic() 
{
    if (DrivetrainActive)
    {
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
    }
    if (balance)
    {
        // Balance fast (theoretical)
        // double roll = deadband(navX.GetRoll(), 3);
        // if (abs(roll)>8)
        // {
        //     onChargingStation = true;
        //     LimitDrivetrainSpeeds(0.3);
        // }
        // if (onChargingStation && abs(roll) < 6 && roll)
        // {
        //     roll = fmax(-roll / 120, -roll/abs(roll)*0.02); // 0.02 is minimum move speed for balance
        // }
        // else{
        //     roll = 1;
        // }

    // Balance slow (tested)
        // double roll = deadband(navX.GetRoll(), 3);
        // if (abs(roll)>8)
        // {
        //     onChargingStation = true;
        //     LimitDrivetrainSpeeds(0.2);
        // }
        // if (onChargingStation)
        // {
        //     roll = -roll / 150;
        // }
        // else{
        //     roll = 1;
        // }
        // autoDesiredStates = swerveKinematics(0, -roll, 0, navX.GetYaw(), false);
    }
    else if(collectCone){
        if (autoIndex != 2 && autoIndex != 3 && autoIndex < 5)
        {
            moveToCoord();
        }
        else if (autoIndex == 2)
        {
            TranslateAlign (Limelight::Cone);
            DrivetrainActive = true;
            SetIntakePosition (PositionPreset::IntakeBottom, (IntakeAtPosition && !deadband(ty, 3)));
            if (IntakeAtPosition && !deadband(ty, 3))
            {
                IntakeAtPosition = false;
                DrivetrainActive = false;

                IntakeUpDown.Set(TalonFXControlMode::PercentOutput, 0);
                IntakeLeader.Set(0.2);
                
                autoIndex = 3;
            }
        }
        else if (autoIndex == 3){
            if(heresyTimer > 60 || IntakeLeader.GetOutputCurrent() > 30)
            {
                DrivetrainActive = false;
                IntakeLeader.Set(0);
            }
            else
            {
                heresyTimer++;
                autoDesiredStates = swerveKinematics(0, -0.5, 0, 0, false);
                DrivetrainActive = true;
            }
        }
        if (!autoIndex)
        {
            SetArmPosition (PositionPreset::ArmBottomHandoff);
            if (ArmAtPosition)
            {
                ArmAtPosition = false;
                ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);
            }
        }
    }
    else if (cubeAutonomous) {
        autostop++;
        frc::SmartDashboard::PutNumber("Autostop", autostop);
        if (autostop < 4000)
        {
            SetArmPosition (PositionPreset::ArmTopHandoff);
            if (ArmAtPosition)
            {
                ArmAtPosition = false;
                
                ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);

                autostop = 4000;
            }
        }
        else
        {
            if (autostop < 8000) // down
            {
                SetArmPosition (PositionPreset::ArmMiddle);
                SetIntakePosition (PositionPreset::IntakeCubeTop, (autostop > 5040));

                if (IntakeAtPosition && ArmAtPosition && autostop < 5000)
                {
                    autostop = 5000;
                }
                
                if(autostop > 5020)
                {
                    IntakeLeader.Set(-1);
                    if (autostop > 5040)
                    {
                        IntakeAtPosition = false;
                        ArmAtPosition = false;
                        
                        IntakeUpDown.Set(TalonFXControlMode::PercentOutput, 0);
                        ArmMotor.Set(TalonFXControlMode::PercentOutput, 0);

                        IntakeLeader.Set(0);
                        
                        autostop = 8000;
                    }
                }
            }
            else 
            {
                SetIntakePosition (1000, IntakeAtPosition);
                if (IntakeAtPosition)
                {
                    autostop = 12000;

                    IntakeAtPosition = false;

                    IntakeUpDown.Set(TalonFXControlMode::PercentOutput, 0);
                    
                    collectCone = true;
                    cubeAutonomous = false;
                }
            }
        }
    }
}

void LimitDrivetrainSpeeds (double speed ) // {0 <= speed <= 1}
{
    limitSpeeds(FLDriveMotor, speed);
    limitSpeeds(FLSwerveMotor, speed);
    limitSpeeds(FRDriveMotor, speed);
    limitSpeeds(FRSwerveMotor, speed);
    limitSpeeds(BLDriveMotor, speed);
    limitSpeeds(BLSwerveMotor, speed);
    limitSpeeds(BRDriveMotor, speed);
    limitSpeeds(BRSwerveMotor, speed);
} 

void Robot::TeleopInit() 
{
    mathConst::rotationVectorMultiplier = 0.4;
    LimitDrivetrainSpeeds(mathConst::speedLimit);
    navX.ZeroYaw();
    navX.SetAngleAdjustment(180.0);
    IntakeUpDown.SetNeutralMode(NeutralMode::Brake);
}
void Robot::TeleopPeriodic()
{
    frc::SmartDashboard::PutNumber("rotvect", mathConst::rotationVectorMultiplier);

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
        alignSouth = deadband(fmod((180-navX.GetAngle()), 360.0), 3)/5;
        if (alignSouth)
        {
            if (deadband(alignSouth*16, 1))
            {
                alignSouth = alignSouth*4;
            }
            moduleDesiredStates = swerveKinematics(0, 0, alignSouth, navX.GetAngle(), false);
        }
        else 
        {
            moduleDesiredStates = swerveKinematics(LLAlignTranslation, 0, 0, navX.GetAngle(), false);
        }
    }
    else if (controller.GetAButton())
    {
        // ALIGN EAST
        alignEast = deadband(fmod((90-navX.GetAngle()), 360.0), 3);
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
    
    if (controllerAux.GetBButton())
    {
        intakeStage = 0;
    }

    if (ArmMotor.GetOutputCurrent()>35 || !armLimitSwitch.Get() || controllerAux.GetBButton())
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
                armPos = -45000;
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

        frc::SmartDashboard::PutNumber("intake stage", intakeStage);

        if (controllerAux.GetAButton())
        {
            if (!intakeStage && ArmMotor.GetSelectedSensorPosition() > (-25000))
            {
                armState = 3;
            }
            else
            {
                if (IntakeUpDown.GetSelectedSensorPosition()<3000 && !IntakeUpDown.GetSelectedSensorVelocity())
                {
                    if (ArmMotor.GetSelectedSensorPosition()>(-10000) && !armState)
                    {
                        intakeStage++;
                        if (intakeStage < 50)
                        {
                            ShooterTop.Set(ControlMode::PercentOutput, -0.4);
                            ShooterBottom.Set(ControlMode::PercentOutput, -0.4);
                            // IntakeLeader.Set(-0.2);
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
                    }
                }
                else
                {
                    intakePos = 3500;
                    intakeStage = 1;
                }
            }
        }
    }

    if (IntakeUpDown.GetOutputCurrent()>37)
    {
        intakePercentage = 0;
        intakePos = IntakeUpDown.GetSelectedSensorPosition();
    }
    else // down
    {
        intakePercentage = (intakePos-IntakeUpDown.GetSelectedSensorPosition());
        frc::SmartDashboard::PutNumber("Intake Error", intakePos-IntakeUpDown.GetSelectedSensorPosition());
        if (abs(intakePos-IntakeUpDown.GetSelectedSensorPosition()) < 500)
        {
            intakePercentage = -0.065;
        }
        else if (intakePos-IntakeUpDown.GetSelectedSensorPosition() < 0)
        {
            intakePercentage = fmin(-0.1, intakePercentage / 50000);
        }
        else 
        {
            intakePercentage = intakePercentage / 80000;
        }
    }
    frc::SmartDashboard::PutNumber("percentage intake", intakePercentage);
    IntakeUpDown.Set(TalonFXControlMode::PercentOutput, intakePercentage);

    if(controller.GetYButtonPressed()){ // up
        intakePos = 3500;
    }
    else if (controller.GetXButtonPressed()){ // down
        intakePos = 27000;
    }
    else if (controllerAux.GetLeftTriggerAxis()>0.5)
    {
        intakePos = 8000;
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
    
    if (controller.GetLeftBumper())
    {
        LED.Set(0.73); // lime
    }
    else if (controller.GetPOV()!=-1)
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
            LimitDrivetrainSpeeds(0.5);
            throttle = true;
        }
    }
    else
    {
        if(throttle==true)
        {
            LimitDrivetrainSpeeds(mathConst::speedLimit);
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