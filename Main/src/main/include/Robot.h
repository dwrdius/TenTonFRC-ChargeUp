#ifndef ROBOT_H
#define ROBOT_H

#include <cmath>
#include <string>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/TimedRobot.h>
#include <ctre/phoenix/sensors/WPI_CANCoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/length.h>
#include <units/angle.h>

#include "logValues.h"
#include "helper.h"

#include "DeviceManager.h"

class Robot : public frc::TimedRobot {
    public:
        void RobotInit() override;
        void RobotPeriodic() override;

        void AutonomousInit() override;
        void AutonomousPeriodic() override;

        void TeleopInit() override;
        void TeleopPeriodic() override;

        void DisabledInit() override;
        void DisabledPeriodic() override;

        void TestInit() override;
        void TestPeriodic() override;

        void SimulationInit() override;
        void SimulationPeriodic() override;
};

#endif