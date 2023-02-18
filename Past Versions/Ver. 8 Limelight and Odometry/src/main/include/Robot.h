#pragma once

#include <frc/TimedRobot.h>

#include <cmath>
#include <string>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/geometry/Rotation2d.h>
#include <units/length.h>
#include <units/angle.h>

#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include <AHRS.h>
#include <ctre/phoenix/sensors/WPI_CANCoder.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "logValues.h"
#include "constants.h"
#include "helper.h"

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

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