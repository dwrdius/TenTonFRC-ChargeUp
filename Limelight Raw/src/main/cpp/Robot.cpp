#include "Robot.h"

// Initialize devices.

// Main Controller
frc::XboxController controller{ControllerIDs::kControllerMainID};

// LED
frc::PWMSparkMax LED{RevIDs::kLED};

// Gyro
AHRS navX{frc::SPI::kMXP};

// Limelight shenanigans
std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
std::string mode = "botpose";
int pipeline;
double tx;
double ty;
double ts;
std::vector<double> aprilPos;

double angleToGoalDegrees;
double distanceFromLimelightToGoalInches;

double LimelightDifference=1;
double LimelightSlew;

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

void Robot::RobotInit() 
{
    processBaseDimensions(mathConst::xCoords, mathConst::yCoords);
    navX.ZeroYaw();
}
void Robot::RobotPeriodic() 
{}

void Robot::AutonomousInit() 
{}

void Robot::AutonomousPeriodic() 
{}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic()
{
    tx = table -> GetNumber("tx", 0.0);
    ty = table -> GetNumber("ty", 0.0);
    ts = table -> GetNumber("ts", 0.0);
    frc::SmartDashboard::PutNumber("ts", ts);
    frc::SmartDashboard::PutNumber("tx", tx);
    frc::SmartDashboard::PutNumber("ty", ty);
    try{
        aprilPos = table -> GetNumberArray(mode, std::vector<double>(6));
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

    if (controller.GetAButton())
    {
        mode = "targetpose_cameraspace";
    }
    else if (controller.GetBButton())
    {
        mode = "target-pose_cameraspace";
    }
    else if (controller.GetYButton())
    {
        mode = "targetpose cameraspace";
    }
    else if (controller.GetXButton())
    {
        mode = "target-pose cameraspace";
    }
    else if (controller.GetRightTriggerAxis())
    {
        mode = "botpose";
    }







    //Retroreflective Tape Tracking
    if (controller.GetLeftBumperPressed()) {
        table -> PutNumber("pipeline", 1);
    }
    //Apriltag tracking
    else if (controller.GetRightBumperPressed())
    {
        table -> PutNumber("pipeline", 2);
    }
    //Intake's intake mechanism
    frc::SmartDashboard::PutNumber("Dist", distanceFromLimelightToGoalInches);

    // Gyro angle (degrees)
    frc::SmartDashboard::PutNumber("Yaw", navX.GetAngle());
    frc::SmartDashboard::PutNumber("Roll", navX.GetRoll());
    frc::SmartDashboard::PutNumber("Pitch", navX.GetPitch());
    frc::SmartDashboard::PutNumber("Rate", navX.GetRate());
}

void Robot::TestInit() {} 
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif