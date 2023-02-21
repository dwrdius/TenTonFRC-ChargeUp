#pragma once

#include <frc/TimedRobot.h>

// std
#include <cmath>
#include <string>
#include <chrono>

// swerve
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/geometry/Rotation2d.h>
#include <units/length.h>
#include <units/angle.h>

// motors
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>

// xbox, navx (gyro), cancoder (encoder)
#include <frc/XboxController.h>
#include <AHRS.h>
#include <ctre/phoenix/sensors/WPI_CANCoder.h>

// log outputs
#include <frc/smartdashboard/SmartDashboard.h>

// include files
#include "logValues.h"
#include "constants.h"
#include "helper.h"
#include "autonomous.h"

// ?????????????????????????????????????????????????????????????????????????????????????????????????
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

// LED
// https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf for colour config
#include <frc/motorcontrol/PWMSparkMax.h>

#include <rev/CANSparkMax.h>

class Robot : public frc::TimedRobot {
    public:
        void RobotInit() override;
        void RobotPeriodic() override;

        void AutonomousInit() override;
        void AutonomousPeriodic() override;

        void TeleopInit() override;
        void TeleopPeriodic() override;

        void TestInit() override;
        void TestPeriodic() override;
};