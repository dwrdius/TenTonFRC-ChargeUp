#ifndef MATHSTUFF_H
#define MATHSTUFF_H

#include <cmath>

#include <frc/geometry/Translation2d.h>

namespace ControllerIDs {
    //USB port addresses on drivestation PC.
    constexpr int kControllerMainID = 0;
    constexpr int kControllerAuxID = 1;
}

namespace mathConst {
    const double rotationVectorMultiplier = 1.2; // controls how much of the vector addition is dedicated to rotation vs field movement  0 < x < double limit idk
    const double speedLimit = 0.2;             // limit motor speed output   0 < x <= 1
    const double driveExponent = 2;

    const double baseWidth = 14.5;             // inches
    const double baseLength = 23.25;

    // speed of swivel
    const double swerveMotorSpeed = 0.5;

    const double slew = 0.1;

    const double swerveDeadband = 0.005;
    const double deadband = 0.2; // DO NOT DECREASE (controller drift was insane)
    const double deadbandOffset = 1/(1-deadband);

    // For determining motor positions on an x,y grid
    const double relativeX = baseWidth / 2;
    const double relativeY = baseLength / 2;

    // relative coordinate of centre of wheels to the centre of the robot [FL, FR, BL, FR]
    double xCoords[4] = {-7.75, 7.75, -7.75, 7.75};
    double yCoords[4] = {12.5, 12.5, -12.5, -12.5};
}

namespace Limelight {
    // how many degrees back is your limelight rotated from perfectly vertical?
    const double limelightMountAngleDegrees = -28.28;

    // distance from the center of the Limelight lens to the floor
    const double limelightLensHeightInches = 41.75;

    // distance from the target to the floor
    const double goalHeightInches = 23.5;

    const double LLDistanceDead = 5; //inches (please don't choose 1)
    const double LLAngleDead = 2; //degrees (0-28)
}

namespace CanIDs {
    //CAN IDs
    const int kFLDriveMotor = 3;
    const int kFRDriveMotor = 1;
    const int kBLDriveMotor = 2;
    const int kBRDriveMotor = 4;

    // Swerve Motors
    const int kFLSwerveMotor = 7;
    const int kFRSwerveMotor = 5;
    const int kBLSwerveMotor = 6;
    const int kBRSwerveMotor = 8;

    // Encoders
    const int kFLCANCoder = 11;
    const int kFRCANCoder = 9;
    const int kBLCANCoder = 10;
    const int kBRCANCoder = 12;
}
namespace CANCoderOffsets {
        constexpr double kFrontLeft{55.810547};
        constexpr double kFrontRight{84.111328};
        constexpr double kBackLeft{56.074219};
        constexpr double kBackRight{-116.015625};
}

namespace Odometry {
    const frc::Translation2d kFLLocation{12.5_in, 7.75_in};
    const frc::Translation2d kFRLocation{12.5_in, -7.75_in};
    const frc::Translation2d kBLLocation{-12.5_in, 7.75_in};
    const frc::Translation2d kBRLocation{-12.5_in, -7.75_in};
}
#endif