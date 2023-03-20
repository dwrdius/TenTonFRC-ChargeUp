#include <cmath>

#include <AHRS.h>
#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>

// Limelight libraries
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
//#include "wpi/span.h"

// LED
// https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf for colour config
#include <frc/motorcontrol/PWMSparkMax.h>

// Colour
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include <frc/util/Color.h>
#include <frc/I2C.h>

// Neo
#include <rev/CANSparkMax.h>

frc::XboxController controller{0};
frc::XboxController ControllerAux{1};

// TalonFX falcon{1};

rev::CANSparkMax IntakeMaster{13, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
rev::CANSparkMax IntakeSlave{14, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

// Gyro
AHRS navX{frc::SPI::kMXP};

// LED
// frc::PWMSparkMax LED{1};

// Colour Sensor
// rev::ColorSensorV3 m_colorSensor{frc::I2C::Port::kOnboard};

// static const frc::Color KYellowTarget = frc::Color(0.4, 0.45, 0.1);
// static const frc::Color KPurpleTarget = frc::Color(0.27, 0.4, 0.2);

double dead(double x)
{
  if(abs(x)<0.2)
  {
    return 0;
  }
  return x/2;
}

double x;

class Robot : public frc::TimedRobot {
 public:
  void TeleopInit() override {
    IntakeMaster.RestoreFactoryDefaults();
    IntakeSlave.RestoreFactoryDefaults();
    IntakeSlave.Follow(IntakeMaster, true);
    // falcon.ConfigPeakOutputForward(0.2);
    // falcon.SetSelectedSensorPosition(0);
    // falcon.SetNeutralMode(NeutralMode::Brake);

  }
  void TeleopPeriodic() override {
    if(controller.GetAButton())
    {
      IntakeMaster.Set(0.4);
      // falcon.Set(TalonFXControlMode::Position, 3000);

    }
    else if (controller.GetBButton())
    {
      IntakeMaster.Set(-1);
      // x = falcon.GetSelectedSensorPosition();
      // falcon.Set(TalonFXControlMode::PercentOutput, (30000-x)/20000);
    }
    else
    {
      IntakeMaster.Set(0);
    }
    // frc::SmartDashboard::PutNumber("percent", (30000-x)/20000);
    // frc::SmartDashboard::PutNumber("position", falcon.GetSelectedSensorPosition());
    // frc::SmartDashboard::PutNumber("dklfjsd", navX.GetYaw());
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif